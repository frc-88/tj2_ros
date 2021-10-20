import os
import time
import queue
import socket
import select
import struct
import logging
import threading

from logger import make_logger
from tunnel_protocol import TunnelProtocol

logger = make_logger("server", logging.DEBUG)


class TunnelServer:
    def __init__(self, address, port):
        self.address = address
        self.port = port
        self.device = None

        self.protocol = TunnelProtocol()

        self.should_stop = False
        self.device_paused = False

        self.read_task = threading.Thread(target=self.read_task_fn)
        self.read_thread_exception = None

        self.read_update_rate_hz = 60.0
        self.update_delay = 1.0 / self.read_update_rate_hz

        self.prev_packet_time = 0.0

        self.prev_packet_num_report_time = 0.0

        self.read_buffer = b''

        self.packet_write_timeout = 1.0
        self.packet_read_timeout = 1.0

        self.ready_keyword = "swerve"

        self.inputs = []
        self.outputs = []
        self.message_queues = {}

        self.categories = {
            "?": "s",
            "!": "s",
            "txrx": "dd"
        }

    def start(self):
        logger.info("Starting socket server")

        self.device = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.device.setblocking(0)
        self.device.bind((self.address, self.port))
        self.device.listen(5)

        self.inputs.append(self.device)

        self.read_task.start()

    def packet_callback(self, category):
        pass

    def update(self):
        if self.read_thread_exception is not None:
            logger.error("Error detected in read task. Raising exception")
            raise self.read_thread_exception

        current_time = time.time()
        if current_time - self.prev_packet_num_report_time > 60.0:
            logger.info("Packet numbers: Read: %s. Write: %s" % (
                self.protocol.read_packet_num, self.protocol.write_packet_num
            ))
            self.prev_packet_num_report_time = current_time

    def stop(self):
        if self.should_stop:
            logger.info("Stop flag already set")
            return
        logger.info("Stopping interface")
        self.should_stop = True
        logger.info("Set stop flag")
        self.pre_stop_callback()
        time.sleep(0.1)
        self.device.close()
        logger.info("Device connection closed")
    
    def pre_stop_callback(self):
        pass
    
    def read_task_fn(self):
        self.prev_packet_time = time.time()

        while True:
            time.sleep(self.update_delay)
            if self.should_stop:
                logger.info("Exiting read thread")
                return

            if len(self.inputs) == 0:
                logger.info("Socket server is indicating it should exit")
                return
                
            self.poll_socket()
    
    def write(self, category, *args):
        packet = self.protocol.make_packet(category, *args)
        logger.debug("Writing %s" % repr(packet))
        for message_queue in self.message_queues.values():
            message_queue.put(packet)

    def readline(self, stream):
        data = b''
        char = b''
        begin_time = time.time()
        while char != self.protocol.PACKET_STOP:
            if time.time() - begin_time > self.packet_read_timeout:
                break
            char = stream.recv(1)
            data += char
        return data

    def poll_socket(self):
        readable, writable, exceptional = select.select(self.inputs, self.outputs, self.inputs, self.packet_read_timeout)
        for stream in readable:
            if stream is self.device:
                connection, client_address = stream.accept()
                connection.setblocking(0)
                self.inputs.append(connection)
                self.message_queues[connection] = queue.Queue()
                logger.info("Registering client '%s'" % str(client_address))
            else:
                logger.debug("Reading from socket")
                data = self.readline(stream)
                logger.debug("Got data: %s" % repr(data))
                if data:
                    self.parse_packet(data)
                    if stream not in self.outputs:
                        self.outputs.append(stream)
                else:
                    if stream in self.outputs:
                        self.outputs.remove(stream)
                    self.inputs.remove(stream)
                    stream.close()
                    del self.message_queues[stream]
                    logger.info("Removing client")

        for stream in readable:
            if stream is self.device:
                continue
            message_queue = self.message_queues[stream]
            if message_queue.empty():
                continue
            next_msg = self.message_queues[stream].get_nowait()
            stream.send(next_msg)
        # for stream in writable:
        #     try:
        #         next_msg = self.message_queues[stream].get_nowait()
        #     except queue.Empty:
        #         self.outputs.remove(stream)
        #     else:
        #         stream.send(next_msg)
        

        for stream in exceptional:
            self.inputs.remove(stream)
            if stream in self.outputs:
                self.outputs.remove(stream)
            stream.close()
            del self.message_queues[stream]
            logger.info("Client had an exception")
    
    def parse_packet(self, packet):
        code, recv_time, data = self.protocol.parse_packet(packet, self.categories)
        if self.protocol.is_code_warning(code):
            category = data[0]
            data = data[1:]
            if category == "?":
                self.write("ready", time.time(), self.ready_keyword)


    def set_start_time(self, device_time):
        self.device_start_time = time.time()
        self.offset_time = device_time

    def get_device_time(self, device_time):
        return self.device_start_time + device_time - self.offset_time

    def pause_device(self):
        logger.info("Pausing device")
        self.device_paused = True

    def resume_device(self):
        logger.info("Resuming device")
        self.device_paused = False


class TestServer(TunnelServer):
    def __init__(self, address, port):
        super().__init__(address, port)
    
    def packet_callback(self, category, recv_time, data):
        pass

def test_write(interface):
    while True:
        if interface.should_stop:
            return
        interface.write("test", "something", 0.88, 10, "else")
        time.sleep(1.0)


def main():
    interface = TestServer("127.0.0.1", 50000)
    write_thread = threading.Thread(target=test_write, args=(interface,))

    write_thread.start()
    try:
        interface.start()
        while True:
            interface.update()
            time.sleep(0.05)
    finally:
        interface.stop()


if __name__ == '__main__':
    main()
