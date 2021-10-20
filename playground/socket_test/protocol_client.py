import os
import time
import socket
import select
import struct
import threading
import logging

from logger import make_logger
from tunnel_protocol import TunnelProtocol

logger = make_logger("client", logging.DEBUG)


class TunnelClient:
    def __init__(self, address, port):
        self.address = address
        self.port = port
        self.device = None

        self.categories = {
            "ready": "ds",
            "txrx": "dd"
        }

        self.protocol = TunnelProtocol()

        self.should_stop = False
        self.device_paused = False

        self.read_task = threading.Thread(target=self.read_task_fn)
        self.read_thread_exception = None

        self.read_update_rate_hz = 60.0
        self.update_delay = 1.0 / self.read_update_rate_hz

        self.prev_packet_time = 0.0

        self.prev_packet_num_report_time = 0.0

        self.write_lock = threading.Lock()
        self.read_lock = threading.Lock()

        self.ready_state = {
            "name": "",
            "is_ready": False,
            "time": 0.0
        }

        self.read_buffer = b''

        self.initial_connect_delay = 2.0
        self.check_ready_timeout = 10.0
        self.write_timeout = 1.0
        self.packet_read_timeout = 10.0
        self.packet_ok_timeout = 10.0

        self.ready_keyword = "swerve"

        self.device_start_time = 0.0
        self.offset_time = 0

        self.is_active = False

        self.wait_for_ok_reqs = {}

        self.large_packets = {}

    def start(self):
        logger.info("Starting socket interface")

        self.device = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.device.connect((self.address, self.port))
        time.sleep(self.initial_connect_delay)

        self.read_task.start()
        logger.info("Read thread started")
        time.sleep(1.0)

        self.check_ready()

    def process_packet(self, category, recv_time, data):
        if category == "txrx":
            packet_num = data[0]
            error_code = data[1]

            if packet_num in self.wait_for_ok_reqs:
                self.wait_for_ok_reqs[packet_num] = error_code
                logger.debug("txrx ok_req %s: %s" % (packet_num, error_code))

            if self.protocol.is_code_warning(error_code):
                self.log_packet_error_code(error_code, packet_num)
            else:
                logger.debug("No error in transmitted packet #%s" % packet_num)

            # if error_code == 4:  # mismatched checksum
            #     if self.parse_next_segment("s"):
            #         logger.warn("mismatched checksum packet: %s" % self.parsed_data[2])

        elif category == "ready":
            self.ready_state["time"] = data[0]
            self.ready_state["name"] = data[1]
            self.ready_state["is_ready"] = True

            logger.info("Ready signal received! %s" % self.ready_state)
        
        else:
            self.packet_callback(category, recv_time, data)
    
    def packet_callback(self, category):
        pass

    def check_ready(self):
        logger.info("Checking if device is ready")
        self.write("?", self.ready_keyword)

        begin_time = time.time()
        write_time = time.time()

        while not self.ready_state["is_ready"]:
            if time.time() - begin_time > self.check_ready_timeout:
                break
            if time.time() - write_time > self.write_timeout:
                logger.info("Writing ready signal again")
                self.write("?", self.ready_keyword)
                write_time = time.time()

            if self.is_available():
                self.read()

        if self.ready_state["is_ready"]:
            self.set_start_time(self.ready_state["time"])
            logger.info("Device is ready. Device name is %s" % self.ready_state["name"])
        else:
            raise Exception("Failed to receive ready signal within %ss" % self.check_ready_timeout)

    def write_stop(self):
        self.write("!", self.ready_keyword)

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

        self.write_stop()

        with self.read_lock:
            self.device.close()
        logger.info("Device connection closed")

    def pre_stop_callback(self):
        pass

    def wait_for_ok(self, packet_num=None):
        if packet_num is None:
            packet_num = self.write_packet_num - 1
        self.wait_for_ok_reqs[packet_num] = None
        start_timer = time.time()
        while True:
            if time.time() - start_timer > self.packet_ok_timeout:
                logger.warn("Timed out while waiting for response from packet #%s" % packet_num)
                return False
            if self.wait_for_ok_reqs[packet_num] is not None:
                error_code = self.wait_for_ok_reqs.pop(packet_num)
                logger.debug("Received response for packet #%s: %s" % (packet_num, error_code))
                return error_code == 0 or error_code == 6
            time.sleep(0.01)

    def is_available(self):
        readable, writable, exceptional = select.select([self.device], [], [])
        for stream in readable:
            if stream is self.device:
                return True
        return False
        
    def write_txrx(self, code):
        self.write("txrx", self.protocol.read_packet_num, code)

    def write_large(self, name, destination, arg):
        assert type(arg) == str or type(arg) == bytes
        if type(arg) == str:
            arg = arg.encode()

        segments = []
        for index in range(0, len(arg), self.max_segment_len):
            offset = min(self.max_segment_len, len(arg) - index)
            segments.append(arg[index: index + offset])
        num_segments = len(segments)
        for index, segment in enumerate(segments):
            self.write(name, destination, index, num_segments, segment)
            if not self.wait_for_ok():
                logger.warn("Failed to receive ok signal on segment %s of %s. %s" % (index + 1, num_segments, name))
                break
            else:
                logger.info("Uploaded segment #%s of %s" % (index + 1, num_segments))

    def write(self, category, *args):
        self.write_lock.acquire()

        if self.device_paused:
            logger.debug("Device is paused. Skipping write: %s, %s" % (str(name), str(args)))
            return

        packet = self.protocol.make_packet(category, *args)
        self._write_packet(packet)

        self.write_lock.release()

    def _write_packet(self, packet):
        logger.debug("Writing %s" % str(packet))
        try:
            self.device.sendall(packet)
        except BaseException as e:
            logger.error("Exception while writing packet %s: %s" % (packet, str(e)), exc_info=True)

    def wait_for_packet_start(self):
        begin_time = time.time()
        msg_buffer = b""
        c1 = ""
        c2 = ""

        while True:
            if time.time() - begin_time > self.packet_read_timeout:
                return False

            if not self.is_available():
                continue

            c1 = self.device.recv(1)
            if c1 == self.protocol.PACKET_START_0:
                c2 = self.device.recv(1)
                if c2 == self.protocol.PACKET_START_1:
                    return True
            elif c1 == self.protocol.PACKET_STOP:
                logger.info("Device message: %s" % (msg_buffer))
                msg_buffer = b""
            else:
                msg_buffer += c1

    def read(self):
        with self.read_lock:
            return self._read()

    def readline(self):
        begin_time = time.time()
        buffer = b""
        len_bytes = b''
        packet_len = 0
        counter = 0
        while True:
            if time.time() - begin_time > self.packet_read_timeout:
                break
            if not self.is_available():
                continue
            c = self.device.recv(1)
            if len(len_bytes) < 2:
                len_bytes += c
                if len(len_bytes) == 2:
                    packet_len = int.from_bytes(len_bytes, "big")
                continue

            counter += 1
            if counter > packet_len:
                if c != self.protocol.PACKET_STOP:
                    logger.error("Packet didn't end with stop character: %s. buffer: %s" % (c, buffer))
                    return None
                break
            buffer += c
        return buffer

    def _read(self):
        # if self.device_paused:
        #     logger.debug("Device is paused. Skipping read")
        #     return

        if not self.wait_for_packet_start():
            return

        buffer = self.readline()
        code, recv_time, data = self.protocol.parse_packet(buffer, self.categories)
        
        if self.protocol.is_code_warning(code):
            try:
                category = data[0]
                data = data[1:]
                self.process_packet(category, recv_time, data)
            except BaseException as e:
                logger.error("Exception while processing packet %s" % (str(e)), exc_info=True)
                logger.error("Error packet: %s" % self.read_buffer)
        self.write_txrx(code)

    def read_task_fn(self):
        self.prev_packet_time = time.time()

        while True:
            time.sleep(self.update_delay)
            if self.should_stop:
                logger.info("Exiting read thread")
                return

            while self.is_available():
                self.read()

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


class TestClient(TunnelClient):
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
    interface = TestClient("127.0.0.1", 50000)

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
