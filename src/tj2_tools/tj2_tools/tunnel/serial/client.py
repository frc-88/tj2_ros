import rospy
import queue
import serial
import threading

from ..protocol import TunnelProtocol


class TunnelSerialClient:
    def __init__(self, address, baud):
        self.protocol = TunnelProtocol()

        self.buffer = b''

        self.inputs = []
        self.outputs = []
        self.message_queue = queue.Queue(maxsize=100)
        
        self.device = serial.Serial(address, baud)
        self.device.nonblocking()

        self.poll_timeout = 1.0
        self.read_block_size = 1024

        self.write_lock = threading.Lock()

    def update(self):
        if self.device.in_waiting == 0:
            return
        recv_msg = self.device.read(self.device.in_waiting)
        self.buffer += recv_msg
        self.buffer, results = self.protocol.parse_buffer(self.buffer)
        for result in results:
            if not self.protocol.is_code_error(result.error_code):
                continue
            category = result.category
            if category == "__msg__":
                rospy.loginfo("Tunnel message: %s" % result.get_string())
            else:
                self.packet_callback(result)

    def write(self, category, *args):
        packet = self.protocol.make_packet(category, *args)
        with self.write_lock:
            self.device.write(packet)

    
    def packet_callback(self, result):
        pass

    def stop(self):
        self.device.close()

