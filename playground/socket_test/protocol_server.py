import time
import queue
import socket
import select
import random
import logging
import threading

from logger import make_logger
from tunnel_protocol import TunnelProtocol

logger = make_logger("server", logging.INFO)


class TunnelServer:
    def __init__(self, address, port):
        self.address = address
        self.port = port
        self.device = None

        self.categories = {
            "ping": "f",
            "cmd": "fff"
        }

        self.protocol = TunnelProtocol()

        self.inputs = []
        self.outputs = []
        self.message_queues = {}

        self.poll_timeout = 1.0

        self.buffer = b''

    def start(self):
        self.device = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.device.setblocking(0)
        self.device.bind((self.address, self.port))
        self.device.listen(5)
        self.inputs.append(self.device)
    
    def poll_socket(self):
        if len(self.inputs) == 0:
            return False

        readable, writable, exceptional = select.select(self.inputs, self.outputs, self.inputs, self.poll_timeout)
        for stream in readable:
            if stream is self.device:
                connection, client_address = stream.accept()
                connection.setblocking(0)
                self.inputs.append(connection)
                self.message_queues[connection] = queue.Queue()
                logger.info("Registering client %s:%s" % client_address)
                if connection not in self.outputs:
                    self.outputs.append(connection)
            else:
                recv_msg = stream.recv(1024)
                self.buffer += recv_msg
                if recv_msg:
                    if len(self.buffer) > 0:
                        logger.debug("Buffer: %s" % repr(self.buffer))
                        self.buffer, results = self.protocol.parse_buffer(self.buffer, self.categories)
                        for result in results:
                            error_code, recv_time, data = result
                            category = data[0]
                            data = data[1:]
                            self.packet_callback(error_code, recv_time, category, data)
                    self.message_queues[stream].put(recv_msg)  # echo back to client
                    
                else:
                    self.close_stream(stream)

        for stream in writable:
            try:
                next_msg = self.message_queues[stream].get_nowait()
            except KeyError:
                continue
            except queue.Empty:
                continue
            try:
                logger.debug("Writing: %s" % next_msg)
                stream.send(next_msg)
            except ConnectionResetError:
                self.close_stream(stream)

        for stream in exceptional:
            self.close_stream(stream)
    
    def write(self, category, *args):
        serialized_args = ", ".join(map(str, args))
        
        for message_queue in self.message_queues.values():
            if message_queue.full():
                logger.debug("Discarding write (%s, %s). Queue is full." % (category, serialized_args))
                return
            logger.debug("Creating packet from args: (%s, %s)" % (category, serialized_args))
            packet = self.protocol.make_packet(category, *args)

            logger.debug("Queueing packet: %s" % repr(packet))
            message_queue.put(packet)

    def close_stream(self, stream):
        self.inputs.remove(stream)
        if stream in self.outputs:
            self.outputs.remove(stream)
        stream.close()
        del self.message_queues[stream]

    def update(self):
        self.poll_socket()
    
    def packet_callback(self, error_code, recv_time, category, data):
        if category == "ping":
            self.write("ping", data[0])
        elif category == "cmd":
            logger.info("Command. vx=%0.4f, vy=%0.4f, vt=%0.4f" % data)

    def stop(self):
        self.device.close()

def test_write(interface):
    while True:
        data = [random.random() for _ in range(6)]
        interface.write("odom", *data)
        time.sleep(0.02)


def main():
    interface = TunnelServer("", 5800)
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
