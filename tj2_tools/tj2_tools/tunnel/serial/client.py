import rospy
import queue
import serial
import select
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
    
    def start(self):
        pass

    def update(self):
        readable, writable, exceptional = select.select([self.device], [], [], self.poll_timeout)
        for stream in readable:
            if stream is self.device:
                rospy.logdebug("Reading from serial")
                recv_msg = stream.recv(self.read_block_size)
                rospy.logdebug("Received: %s" % recv_msg)
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
        if self.message_queue.qsize() == 0:
            return
        rospy.logdebug("Queue size: %s" % self.message_queue.qsize())
        with self.write_lock:
            while self.message_queue.qsize():
                rospy.logdebug("Getting from message queue")
                packet = self.message_queue.get()
                rospy.logdebug("Writing: %s" % packet)
                self.device.write(packet)

        for stream in exceptional:
            rospy.loginfo("Closing connection due to an exception")
            self.inputs.remove(stream)
            if stream in self.outputs:
                self.outputs.remove(stream)
            stream.close()
    
    def write(self, category, *args):
        serialized_args = ", ".join(map(str, args))
        
        if self.message_queue.full():
            rospy.logdebug("Discarding write (%s, %s). Queue is full." % (category, serialized_args))
            return
        rospy.logdebug("Creating packet from args: (%s, %s)" % (category, serialized_args))
        packet = self.protocol.make_packet(category, *args)

        with self.write_lock:
            rospy.logdebug("Queueing packet: %s" % repr(packet))
            self.message_queue.put(packet)
    
    def packet_callback(self, result):
        pass

    def stop(self):
        self.device.close()

