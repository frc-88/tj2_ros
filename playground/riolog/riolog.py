import sys
import queue
import socket
import select
import logging
import threading

from logger import make_logger


logger = make_logger("riolog", logging.INFO)


class RioLog:
    # loosely referenced:
    # https://github.com/wpilibsuite/riolog/blob/master/src/main/java/netconsole2/RioConsole.java
    def __init__(self, address):
        self.address = address
        self.port = 1741
        self.device = None


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
                # recv_msg = stream.recv(self.read_block_size)
                # logger.debug("Received: %s" % recv_msg)
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
    

    def write(self, data):
        if self.message_queue.full():
            logger.debug("Discarding write (%s). Queue is full." % str(data))
            return
        logger.debug("Creating packet from args: (%s)" % str(data))

        with self.write_lock:
            logger.debug("Queueing data: %s" % repr(data))
            self.message_queue.put(data)
    
    def stop(self):
        self.device.close()

    def segment_callback(self, tag, data):
        # if tag == 11:
        #     pass
        # elif tag == 12:
        #     pass
        # else:
        #     return  # ignore other tags
        
        logger.info("msg: %s" % repr(data))


def main():
    if len(sys.argv[1]) == 0:
        host = "127.0.0.1"
    else:
        host = sys.argv[1]
    print("Host: %s" % host)

    riolog = RioLog(host)
    try:
        riolog.start()
        while True:
            riolog.update()
    finally:
        riolog.stop()


if __name__ == '__main__':
    main()
