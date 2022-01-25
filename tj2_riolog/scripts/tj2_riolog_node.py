#!/usr/bin/python3
import os
import rospy
import queue
import select
import socket
import paramiko
import threading


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
        lines = remote_file.read().splitlines()
        for line in lines[-min(len(lines), self.max_initial_lines):]:
            yield line.decode()

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

        self.poll_timeout = 1.0
        self.read_block_size = 4096

    def run(self):
        self.start()
        while not rospy.is_shutdown():
            self.update()
            self.rate.sleep()

    def start(self):
        rospy.loginfo("Connecting to %s:%s" % (self.host, self.port))
        self.device = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.device.settimeout(1.0)
        self.device.connect((self.host, self.port))
        self.outputs.append(self.device)
        rospy.loginfo("Connection established")

    def update(self):
        readable, _, exceptional = select.select([], self.outputs, [], self.poll_timeout)
        for stream in readable:
            if stream is self.device:
                rospy.logdebug("Reading from socket")
                # recv_msg = stream.recv(self.read_block_size)
                # rospy.logdebug("Received: %s" % recv_msg)
                length_bytes = stream.recv(2)
                if len(length_bytes) == 0:
                    continue
                length = int.from_bytes(length_bytes, "big")
                if length == 0:
                    continue
                tag = stream.recv(1)
                length -= 1  # subtract 1 for tag
                data = b''
                while len(data) < length:
                    data += stream.recv(length)
                    length -= len(data)
                self.segment_callback(tag, data)

        for stream in exceptional:
            rospy.loginfo("Closing connection due to an exception")
            self.inputs.remove(stream)
            if stream in self.outputs:
                self.outputs.remove(stream)
            stream.close()

    def stop(self):
        self.device.close()

    def segment_callback(self, tag, data):
        if tag == 11:
            status = "ERROR"
        elif tag == 12:
            status = "INFO"
        else:
            status = "??"
        
        try:
            rospy.loginfo("[Riolog][%s] %s" % (status, data.decode()))
        except UnicodeDecodeError:
            rospy.loginfo("[Riolog][%s] %s" % (status, repr(data)))


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

        if self.use_ssh:
            self.tailer = SSHTailer(self.host, self.remote_path)
        else:
            self.tailer = RiologTailer(self.host, self.remote_path)

    def get_ssh_tailer(self):
        return SSHTailer(self.host, self.remote_path)
    
    def get_riolog_tailer(self):
        return RiologTailer(self.host, self.remote_path)

    def run(self):
        if self.use_ssh:
            tailer = self.get_ssh_tailer()
            try:
                tailer.run()
                return
            except socket.timeout as e:
                rospy.logwarn("Failed to connect to the rio via SSH. Falling back on socket tailer. %s" % e)

            tailer = self.get_riolog_tailer()
            tailer.run()
        else:
            tailer = self.get_riolog_tailer()
            tailer.run()


if __name__ == "__main__":
    node = TJ2RioLog()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)
