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

        self.write_timeout = 1.0
        self.packet_read_timeout = 10.0
        self.packet_ok_timeout = 10.0

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
        for message_queue in self.message_queues.values():
            message_queue.put(packet)

    def readline(self, stream):
        data = b''
        char = b''
        while char != b'\n':
            char = stream.recv(1)
            data += char
        return data

    def poll_socket(self):
        readable, writable, exceptional = select.select(self.inputs, self.outputs, self.inputs)
        for stream in readable:
            if stream is self.device:
                connection, client_address = stream.accept()
                connection.setblocking(0)
                self.inputs.append(connection)
                self.message_queues[connection] = queue.Queue()
            else:
                data = self.readline(stream)
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

        for stream in writable:
            try:
                next_msg = self.message_queues[stream].get_nowait()
            except queue.Empty:
                # self.outputs.remove(stream)
                pass
            else:
                stream.send(next_msg)

        for stream in exceptional:
            self.inputs.remove(stream)
            if stream in self.outputs:
                self.outputs.remove(stream)
            stream.close()
            del self.message_queues[stream]
    
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
        if category == "power":
            shunt_voltage = data[0]
            bus_voltage = data[1]
            current_mA = data[2]
            power_mW = data[3]
            load_voltage = data[4]

            logger.info(
                "\nshunt_voltage: %0.4f V\n" \
                "bus_voltage: %0.4f V\n" \
                "current_mA: %0.4f mA\n" \
                "power_mW: %0.4f mW\n" \
                "load_voltage: %0.4f V\n" % (
                    shunt_voltage,
                    bus_voltage,
                    current_mA,
                    power_mW,
                    load_voltage
                )
            )

def main():
    interface = TestServer("127.0.0.1", 50000)

    timer = time.time()
    try:
        interface.start()
        while True:
            interface.update()
            time.sleep(0.05)
    finally:
        interface.stop()


if __name__ == '__main__':
    main()
