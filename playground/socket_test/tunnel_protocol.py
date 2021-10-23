import os
from threading import Condition
import time
import struct
import logging

from logger import make_logger

logger = make_logger("protocol", logging.DEBUG)

NO_ERROR = 0
PACKET_0_ERROR = 1
PACKET_1_ERROR = 2
PACKET_TOO_SHORT_ERROR = 3
CHECKSUMS_DONT_MATCH_ERROR = 4
PACKET_COUNT_NOT_FOUND_ERROR = 5
PACKET_COUNT_NOT_SYNCED_ERROR = 6
PACKET_CATEGORY_ERROR = 7
INVALID_FORMAT_ERROR = 8
PACKET_STOP_ERROR = 9
SEGMENT_TOO_LONG_ERROR = 10
PACKET_TIMEOUT_ERROR = 11
FORMAT_TYPE_ERROR = 12


class TunnelProtocol:
    def __init__(self):
        self.PACKET_START_0 = b'\x12'
        self.PACKET_START_1 = b'\x13'
        self.PACKET_STOP = b'\n'
        self.PACKET_SEP = b'\t'
        self.PACKET_SEP_STR = '\t'

        self.max_packet_len = 128
        self.max_segment_len = 64

        self.read_packet_num = -1
        self.recv_packet_num = 0
        self.write_packet_num = 0

        self.buffer_index = 0
        self.current_segment = b''
        self.parse_error_code = -1

        self.packet_error_codes = {
            NO_ERROR: "no error",
            PACKET_0_ERROR: "c1 != %s" % str(self.PACKET_START_0),
            PACKET_1_ERROR: "c2 != %s" % str(self.PACKET_START_1),
            PACKET_TOO_SHORT_ERROR: "packet is too short",
            CHECKSUMS_DONT_MATCH_ERROR: "checksums don't match",
            PACKET_COUNT_NOT_FOUND_ERROR: "packet count segment not found",
            PACKET_COUNT_NOT_SYNCED_ERROR: "packet counts not synchronized",
            PACKET_CATEGORY_ERROR: "failed to find category segment",
            INVALID_FORMAT_ERROR: "invalid format",
            PACKET_STOP_ERROR: "packet didn't end with stop character",
            SEGMENT_TOO_LONG_ERROR: "packet segment is too long",
            PACKET_TIMEOUT_ERROR: "packet receive timed out",
            FORMAT_TYPE_ERROR: "provided formats do not correctly parse the packet"
        }
        self.packet_warning_codes = [
            NO_ERROR,
            PACKET_COUNT_NOT_SYNCED_ERROR,
        ]

        self.minimum_packet = self.make_packet("x")
        self.min_packet_len = len(self.minimum_packet)

    @staticmethod
    def to_uint16_bytes(integer):
        return integer.to_bytes(2, 'big')

    @staticmethod
    def to_int32_bytes(integer):
        return integer.to_bytes(4, 'big', signed=True)

    @staticmethod
    def to_float_bytes(floating_point):
        return struct.pack('d', floating_point)
    
    @staticmethod
    def to_int(raw_bytes):
        return int.from_bytes(raw_bytes, 'big')

    def make_packet(self, category, *args):
        packet = self.packet_header(category)
        for arg in args:
            if type(arg) == int:
                packet += self.to_int32_bytes(arg)
            elif type(arg) == float:
                packet += self.to_float_bytes(arg)
            elif type(arg) == str or type(arg) == bytes:
                assert len(arg) <= self.max_segment_len, arg
                len_bytes = self.to_uint16_bytes(len(arg))
                if type(arg) == str:
                    arg = arg.encode()
                packet += len_bytes + arg
            else:
                logger.warning("Invalid argument type: %s, %s" % (type(arg), arg))

        packet = self.packet_footer(packet)

        self.write_packet_num += 1
        
        return packet

    def packet_header(self, category):
        packet = self.to_int32_bytes(self.write_packet_num)
        packet += str(category).encode() + self.PACKET_SEP
        return packet

    def packet_footer(self, packet):
        calc_checksum = self.calculate_checksum(packet)

        packet += b"%02x" % calc_checksum

        packet_len = len(packet)
        packet_len_bytes = self.to_uint16_bytes(packet_len)

        packet = self.PACKET_START_0 + self.PACKET_START_1 + packet_len_bytes + packet
        packet += self.PACKET_STOP

        return packet
    
    def calculate_checksum(self, packet):
        calc_checksum = 0
        for val in packet:
            calc_checksum += val
        calc_checksum &= 0xff

        return calc_checksum
    
    def extract_checksum(self, packet):
        try:
            return int(packet[-2:], 16)
        except ValueError as e:
            logger.warning("Failed to parse checksum as int: %s" % str(e))
            return -1

    def parse_buffer(self, buffer: bytes, format_mapping):
        results = []
        def get_char(n=1):
            nonlocal buffer
            if n > len(buffer):
                return b''
            c = buffer[0:n]
            buffer = buffer[n:]
            return c

        while True:
            if len(buffer) == 0:
                break
            c = get_char()
            if c != self.PACKET_START_0:
                break
            packet = c
            c = get_char()
            if c != self.PACKET_START_1:
                break
            packet += c
            raw_length = get_char(2)
            if len(raw_length) == 0:
                break
            packet += raw_length
            length = self.to_int(raw_length)
            packet += get_char(length + 1)
            if len(packet) == 0:
                break
            if packet[-1:] != self.PACKET_STOP:
                break

            result = self.parse_packet(packet, format_mapping)
            results.append(result)
        
        return buffer, results

    def parse_packet(self, packet: bytes, format_mapping):
        recv_time = time.time()
        if len(packet) < self.min_packet_len:
            logger.warning("Packet is not the minimum length (%s): %s" % (self.min_packet_len, repr(packet)))
            return PACKET_TOO_SHORT_ERROR, recv_time, []
        
        if packet[0:1] != self.PACKET_START_0:
            logger.warning("Packet does not start with PACKET_START_0: %s" % repr(packet))
            self.read_packet_num += 1
            return PACKET_0_ERROR, recv_time, []
        if packet[1:2] != self.PACKET_START_1:
            logger.warning("Packet does not start with PACKET_START_1: %s" % repr(packet))
            self.read_packet_num += 1
            return PACKET_1_ERROR, recv_time, []
        if packet[-1:] != self.PACKET_STOP:
            logger.warning("Packet does not stop with PACKET_STOP: %s" % repr(packet))
            self.read_packet_num += 1
            return PACKET_STOP_ERROR, recv_time, []
        
        packet = packet[4:-1]  # remove start, length, and stop characters
        calc_checksum = self.calculate_checksum(packet[:-2])
        recv_checksum = self.extract_checksum(packet)
        if recv_checksum != calc_checksum:
            logger.warning("Checksum failed! recv %02x != calc %02x. %s" % (recv_checksum, calc_checksum, repr(packet)))
            self.read_packet_num += 1
            return CHECKSUMS_DONT_MATCH_ERROR, recv_time, []
        
        packet = packet[:-2]  # remove checksum

        self.buffer_index = 0

        # get packet num segment
        if not self.get_next_segment(packet, 4):
            logger.warning("Failed to find packet number segment! %s" % (repr(packet)))
            self.read_packet_num += 1
            return PACKET_COUNT_NOT_FOUND_ERROR, recv_time, []
        self.recv_packet_num = self.to_int(self.current_segment)
        
        self.parse_error_code = -1

        if self.read_packet_num == -1:
            self.read_packet_num = self.recv_packet_num
        if self.recv_packet_num != self.read_packet_num:
            logger.warning("Received packet num doesn't match local count. "
                           "recv %s != local %s", self.recv_packet_num, self.read_packet_num)
            logger.debug("Buffer: %s" % packet)
            self.read_packet_num = self.recv_packet_num
            self.set_error_code(PACKET_COUNT_NOT_SYNCED_ERROR)
        logger.debug("Packet num %s passes for %s" % (self.recv_packet_num, str(packet)))

        # find category segment
        if not self.get_next_segment(packet, tab_separated=True):
            logger.warning(
                "Failed to find category segment %s! %s" % (repr(self.current_segment), repr(packet)))
            self.read_packet_num += 1
            return PACKET_CATEGORY_ERROR, recv_time, []
        try:
            category = self.current_segment.decode()
        except UnicodeDecodeError:
            logger.warning("Category segment contains invalid characters: %s, %s" % (
                repr(self.current_segment), repr(packet)))
            self.read_packet_num += 1
            return PACKET_CATEGORY_ERROR, recv_time, []
        
        if len(category) == 0:
            logger.warning("Category segment is empty: %s, %s" % (
                repr(self.current_segment), repr(packet)))
            self.read_packet_num += 1
            return PACKET_CATEGORY_ERROR, recv_time, []
        
        logger.debug("Category '%s' found in %s" % (category, str(packet)))

        if category not in format_mapping:
            logger.warning("'%s' is not applied category: %s" % (
                category, tuple(format_mapping.keys())))
            self.read_packet_num += 1
            return PACKET_CATEGORY_ERROR, recv_time, []
        formats = format_mapping[category]

        parsed_data = []
        for index, f in enumerate(formats):
            if not self.parse_next_segment(packet, parsed_data, f):
                logger.warning("Failed to parse segment #%s. Buffer: %s" % (index, packet))
                return FORMAT_TYPE_ERROR, recv_time, []
        parsed_data.insert(0, category)
        parsed_data = tuple(parsed_data)

        self.set_error_code(NO_ERROR)
        self.read_packet_num += 1

        return self.parse_error_code, recv_time, parsed_data

    def set_error_code(self, code):
        if self.parse_error_code == -1:
            self.parse_error_code = code
    
    def parse_next_segment(self, buffer, data, f):
        if f == 'd' or f == 'u':
            length = 4
        elif f == 'f':
            length = 8
        else:
            length = None

        if not self.get_next_segment(buffer, length):
            return False

        try:
            if f == 'd' or f == 'u':
                data.append(self.to_int(self.current_segment))
            elif f == 's' or f == 'x':
                data.append(self.current_segment)
            elif f == 'f':
                parsed_float = struct.unpack('d', self.current_segment)
                data.append(parsed_float[0])
        except ValueError:
            logger.warning("Failed to parse segment '%s' as '%s'" % (self.current_segment, f))
            return False

        return True

    def get_next_segment(self, buffer, length=None, tab_separated=False):
        if self.buffer_index >= len(buffer):
            return False
        if tab_separated:
            sep_index = buffer.find(self.PACKET_SEP, self.buffer_index)
            if sep_index == -1:
                self.current_segment = buffer[self.buffer_index:]
                self.buffer_index = len(buffer)
            else:
                self.current_segment = buffer[self.buffer_index: sep_index]
                self.buffer_index = sep_index + 1
            return True
        else:
            if length is None:
                # assume first 2 bytes contain the length
                len_bytes = buffer[self.buffer_index: self.buffer_index + 2]
                length = self.to_int(len_bytes)
                self.buffer_index += 2
                if length >= len(buffer):
                    logger.error("Parsed length %s exceeds buffer length! %s" % (length, len(buffer)))
            self.current_segment = buffer[self.buffer_index: self.buffer_index + length]
            self.buffer_index += length
            return True

    def is_code_warning(self, error_code):
        return error_code in self.packet_warning_codes

    def log_packet_error_code(self, error_code, packet_num=None):
        if packet_num is None:
            packet_num = self.read_packet_num
        if error_code == NO_ERROR:
            logger.debug("Packet %s has no error" % packet_num)
            return
        
        if self.is_code_warning(error_code):
            logger.warning("Packet %s returned a warning:" % packet_num)
        else:
            logger.warning("Packet %s returned an error:" % packet_num)
        if error_code in self.packet_error_codes:
            logger.warning("\t%s" % self.packet_error_codes[error_code])
        else:
            logger.warning("\tUnknown error code: %s" % error_code)

if __name__ == '__main__':
    def main():
        protocol = TunnelProtocol()

        print(protocol.minimum_packet)
        code, recv_time, result = protocol.parse_packet(protocol.minimum_packet, {'x': ''})
        protocol.log_packet_error_code(code)
        print(result)
                
        data = "something", b"else", 2.0, 4
        test_packet = protocol.make_packet(*data)
        code, recv_time, result = protocol.parse_packet(test_packet, {"something": 'sfd'})
        assert data == result, "%s != %s" % (data, result)
        protocol.log_packet_error_code(code)
        print(result)


    main()
