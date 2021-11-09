import sys
import time
import queue
import socket
import select
import random
import logging
import threading
import numpy as np

from logger import make_logger
from tunnel_protocol import TunnelProtocol

logger = make_logger("client", logging.INFO)


class TunnelClient:
    def __init__(self, address, port):
        self.address = address
        self.port = port
        self.device = None

        self.categories = {
            "ping": "f",
            "odom": "ffffff",
            "__msg__": "s",
        }

        self.protocol = TunnelProtocol()

        self.buffer = b''

        self.inputs = []
        self.outputs = []
        self.message_queue = queue.Queue(maxsize=100)

        self.poll_timeout = 1.0
        self.read_block_size = 4096

        self.write_lock = threading.Lock()
    
    def start(self):
        self.device = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.device.connect((self.address, self.port))
        self.inputs.append(self.device)
        self.outputs.append(self.device)

    def update(self):
        readable, writable, exceptional = select.select(self.inputs, self.outputs, self.inputs, self.poll_timeout)
        for stream in readable:
            if stream is self.device:
                logger.debug("Reading from socket")
                recv_msg = stream.recv(self.read_block_size)
                logger.debug("Received: %s" % recv_msg)
                self.buffer += recv_msg
                self.buffer, results = self.protocol.parse_buffer(self.buffer, self.categories)
                for result in results:
                    error_code, recv_time, data = result
                    if not self.protocol.is_code_error(error_code):
                        continue
                    category = data[0]
                    data = data[1:]
                    self.packet_callback(error_code, recv_time, category, data)
        
        for stream in writable:
            if self.message_queue.qsize() == 0:
                continue
            logger.debug("Queue size: %s" % self.message_queue.qsize())
            with self.write_lock:
                while self.message_queue.qsize():
                    logger.debug("Getting from message queue")
                    packet = self.message_queue.get()
                    logger.debug("Writing: %s" % packet)
                    stream.send(packet)

        for stream in exceptional:
            logger.info("Closing connection due to an exception")
            self.inputs.remove(stream)
            if stream in self.outputs:
                self.outputs.remove(stream)
            stream.close()
            del self.message_queues[stream]
    
    def write(self, category, *args):
        serialized_args = ", ".join(map(str, args))
        
        if self.message_queue.full():
            logger.debug("Discarding write (%s, %s). Queue is full." % (category, serialized_args))
            return
        logger.debug("Creating packet from args: (%s, %s)" % (category, serialized_args))
        packet = self.protocol.make_packet(category, *args)

        with self.write_lock:
            logger.debug("Queueing packet: %s" % repr(packet))
            self.message_queue.put(packet)
    
    def packet_callback(self, error_code, recv_time, category, data):
        pass

    def stop(self):
        self.device.close()


def test_write(interface):
    last_cmd_time = time.time()
    while True:
        interface.write("ping", time.time())
        time.sleep(0.1)

        if time.time() - last_cmd_time > 0.5:
            interface.write("cmd", 0.0, 0.25, 0.0)
            last_cmd_time = time.time()

class MyTunnel(TunnelClient):
    def __init__(self, address, port):
        super(MyTunnel, self).__init__(address, port)
        self.pings = []
        self.odoms = []
    
    def packet_callback(self, error_code, recv_time, category, data):
        if category == "__msg__":
            logger.info("Tunnel message: %s" % data[0])
        elif category == "ping":
            dt = time.time() - data[0]
            self.pings.append(dt)
            while len(self.pings) > 10:
                self.pings.pop(0)
            
            logger.info("Ping time: %0.6fs.\tAvg: %0.6f.\tStddev: %0.6f.\tInt. Rate: %0.3f" % (dt, np.mean(self.pings), np.std(self.pings), 1.0 / np.mean(self.pings)))
        elif category == "odom":
            self.odoms.append(recv_time)
            while len(self.odoms) > 10:
                self.odoms.pop(0)
            data = list(data)
            rate = 1.0 / np.mean(np.diff(self.odoms))
            data.insert(0, rate)
            logger.info("Odometry. Rate: %0.4f Hz, x=%0.4f, y=%0.4f, t=%0.4f, vx=%0.4f, vy=%0.4f, vt=%0.4f" % tuple(data))

def main():
    if len(sys.argv[1]) == 0:
        host = "127.0.0.1:3000"
    else:
        host = sys.argv[1]
    print("Host: %s" % host)
    address, port = host.split(":")
    port = int(port)
    interface = MyTunnel(address, port)
    write_thread = threading.Thread(target=test_write, args=(interface,))
    write_thread.daemon = True
    write_thread.start()

    try:
        interface.start()
        while True:
            interface.update()
    finally:
        interface.stop()


if __name__ == '__main__':
    main()
