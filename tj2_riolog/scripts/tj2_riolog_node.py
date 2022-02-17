#!/usr/bin/python3
import re
import rospy
import queue
import struct
import select
import socket
import paramiko


class Tailer:
    def __init__(self, host, remote_path):
        self.host = host
        self.remote_path = remote_path
        self.rate = rospy.Rate(5)
    
    def run(self):
        raise NotImplementedError


class SSHTailer(Tailer):
    def __init__(self, host, remote_path, max_initial_lines=300):
        super(SSHTailer, self).__init__(host, remote_path)
        self.username = "lvuser"
        self.line_terminators = "\r\n"

        self.client = None
        self.sftp_client = None
        self.remote_file_size = None
        self.max_initial_lines = max_initial_lines

    def run(self):
        self.connect()
        for line in self.tail():
            rospy.loginfo("[Riolog] %s" % line)

    def connect(self):
        rospy.loginfo("Connecting to %s" % self.host)
        # connect to the host
        self.client = paramiko.SSHClient()
        self.client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.client.connect(self.host, username=self.username, password='', allow_agent=False, look_for_keys=False, timeout=1.0)
        # self.client.connect(self.host, username=self.username, password='')

        rospy.loginfo("Opening remote file %s" % self.remote_path)
        # open a connection to the remote file via SFTP
        self.sftp_client = self.client.open_sftp()


    def tail(self):
        remote_file = self.sftp_client.open(self.remote_path, 'r')
        yield remote_file.read().decode()

        while not rospy.is_shutdown():
            fstat = self.sftp_client.stat(self.remote_path)

            # check if we have the file size
            if self.remote_file_size is not None:
                # if the file's grown
                if self.remote_file_size < fstat.st_size:
                    for line in self.get_new_lines():
                        yield line

            self.remote_file_size = fstat.st_size
            self.rate.sleep()
 
    def get_new_lines(self):
        """
        Opens the file and reads any new data from it.
        """

        remote_file = self.sftp_client.open(self.remote_path, 'r')
        # seek to the latest read point in the file
        remote_file.seek(self.remote_file_size, 0)
        # read any new lines from the file
        line = remote_file.readline()
        while line:
            yield line.strip(self.line_terminators)
            line = remote_file.readline()

        remote_file.close()


class RiologTailer(Tailer):
    def __init__(self, host, remote_path):
        super(RiologTailer, self).__init__(host, remote_path)
        
        self.port = 1741
        self.device = None

        self.inputs = []
        self.outputs = []
        self.message_queue = queue.Queue(maxsize=100)

        self.poll_timeout = 0.1
        self.read_block_size = 4096

    def run(self):
        self.start()
        while not rospy.is_shutdown():
            if not self.update():
                self.rate.sleep()

    def start(self):
        self.connect()
    
    def connect(self):
        exception = None
        for _ in range(50):
            rospy.loginfo("Connecting to %s:%s" % (self.host, self.port))
            self.device = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.device.settimeout(1.0)
            try:
                self.device.connect((self.host, self.port))
            except BaseException as e:
                rospy.logwarn("Failed to connect to RoboRIO via socket: %s" % str(e))
                rospy.sleep(1.0)
                exception = e
                continue
            self.inputs.append(self.device)
            self.outputs.append(self.device)
            exception = None
            break
        if exception is not None:
            raise exception
        rospy.loginfo("Connection established")

    def update(self):
        should_reconnect = False
        readable, writable, exceptional = select.select(self.inputs, self.outputs, [], self.poll_timeout)
        for stream in readable:
            if stream is self.device:
                # rospy.loginfo("Reading from socket")
                # recv_msg = stream.recv(self.read_block_size)
                # rospy.loginfo("Received: %s" % recv_msg)
                length_bytes = stream.recv(2)
                if len(length_bytes) == 0:
                    rospy.logwarn("[Riolog] Failed to get message length")
                    should_reconnect = True
                    break
                length = int.from_bytes(length_bytes, "big")
                if length == 0:
                    rospy.logwarn("[Riolog] Message length zero")
                    should_reconnect = True
                    break
                tag_bytes = stream.recv(1)
                if len(tag_bytes) == 0:
                    rospy.logwarn("[Riolog] Tag is empty")
                    should_reconnect = True
                    break
                tag = int.from_bytes(tag_bytes, "big")
                
                timestamp_bytes = stream.recv(4)[::-1]
                if len(timestamp_bytes) == 0:
                    rospy.logwarn("[Riolog] Timestamp didn't parse")
                    should_reconnect = True
                    break
                timestamp = struct.unpack('f', timestamp_bytes)[0]

                sequence_bytes = stream.recv(2)
                if len(sequence_bytes) == 0:
                    rospy.logwarn("[Riolog] Timestamp didn't parse")
                    should_reconnect = True
                    break
                sequence = int.from_bytes(sequence_bytes, "big")
                length -= 7  # subtract for tag, timestamp, and sequence

                data = stream.recv(length)
                if len(data) == 0:
                    continue
                self.segment_callback(tag, timestamp, sequence, data)

        for stream in exceptional:
            rospy.loginfo("Closing connection due to an exception")
            self.inputs.remove(stream)
            if stream in self.outputs:
                self.outputs.remove(stream)
            stream.close()
        
        if should_reconnect:
            rospy.sleep(1.0)
            self.connect()

        if len(readable) == 0:
            return False
        else:
            return True

    def stop(self):
        self.device.close()

    def segment_callback(self, tag, timestamp, sequence, data):
        if tag == 11:
            status = "ERROR"
        elif tag == 12:
            status = "INFO"
        else:
            status = str(tag)
        
        base_message = "[Riolog][%s @ %0.2f. #%s] " % (status, timestamp, sequence)
        try:
            data = data.decode()
            data = re.sub("\s+", " ", data)
            rospy.loginfo(base_message + data)
        except UnicodeDecodeError:
            rospy.loginfo(base_message + repr(data))


class TJ2RioLog:
    def __init__(self):
        self.node_name = "tj2_riolog"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )

        self.use_ssh = rospy.get_param("~use_ssh", False)
        self.host = rospy.get_param("~rio_host", "10.0.88.2")
        self.remote_path = rospy.get_param("~remote_path", "/home/lvuser/FRC_UserProgram.log")

        rospy.loginfo("%s init complete" % self.node_name)

    def get_main_tailer(self):
        if self.use_ssh:
            return SSHTailer(self.host, self.remote_path)
        else:
            return RiologTailer(self.host, self.remote_path)
    
    def get_fallback_tailer(self):
        if self.use_ssh:
            return RiologTailer(self.host, self.remote_path)
        else:
            return SSHTailer(self.host, self.remote_path)

    def run(self):
        tailer = self.get_main_tailer()
        try:
            tailer.run()
            return
        except Exception as e:
            rospy.logwarn("Failed to connect to the rio. %s. Trying fallback tailer" % e)

        tailer = self.get_fallback_tailer()
        tailer.run()


if __name__ == "__main__":
    node = TJ2RioLog()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
