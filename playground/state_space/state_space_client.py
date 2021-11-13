import sys
import csv
import time
import logging
import threading
import numpy as np
from datetime import datetime
from logger import make_logger
from tj2_tools.tunnel.client import TunnelClient


logger = make_logger("state_space", logging.INFO)


class MyTunnel(TunnelClient):
    def __init__(self, address, port, base_dir):
        categories = {
            "ping": "f",
            "odom": "ffffff",
            "module": "d" + "f" * 19,
        }
        super(MyTunnel, self).__init__(address, port, categories)
        self.pings = []
        self.odoms = []

        self.write_thread = threading.Thread(target=self.write_task)
        self.write_thread.daemon = True

        self.header = [
            "timestamp",
            "module index",
            "wheel velocity",
            "module angle",
            "x_hat azimuth",
            "x_hat azimuth velocity",
            "x_hat wheel velocity",
            "observer x_hat azimuth",
            "observer x_hat azimuth velocity",
            "observer x_hat wheel velocity",
            "error azimuth",
            "error azimuth velocity",
            "error wheel velocity",
            "next reference azimuth",
            "next reference azimuth velocity",
            "next reference wheel velocity",
            "reference azimuth",
            "reference azimuth velocity",
            "reference wheel velocity",
            "lo next voltage",
            "hi next voltage",
        ]
        assert len(self.header) - 1 == len(categories["module"])

        self.path = "%s/%s.csv" % (base_dir, datetime.now().strftime("%Y-%m-%dT%H-%M-%S--%f"))
        self.file = open(self.path, 'w')
        self.writer = csv.writer(self.file)
        self.writer.writerow(self.header)

    def start(self):
        super(MyTunnel, self).start()
        # self.write_thread.start()
    
    def stop(self):
        super(MyTunnel, self).stop()
        self.file.close()
        logger.info("Wrote to %s" % self.path)
    
    def write_task(self):
        last_cmd_time = time.time()
        while True:
            self.write("ping", time.time())
            time.sleep(0.1)

            if time.time() - last_cmd_time > 0.5:
                self.write("cmd", 0.0, 0.25, 0.0)
                last_cmd_time = time.time()

    def packet_callback(self, error_code, recv_time, category, data):
        if category == "__msg__":
            logger.info("Tunnel message: %s" % data[0])
        elif category == "ping":
            dt = time.time() - data[0]
            self.pings.append(dt)
            while len(self.pings) > 10:
                self.pings.pop(0)
            
            logger.info("Ping time: %0.6fs.\tAvg: %0.6f.\tStddev: %0.6f.\tInt. Rate: %0.3f" % (dt, np.mean(self.pings), np.std(self.pings), 1.0 / np.mean(self.pings)))
        elif category == "odom":
            self.odoms.append(recv_time)
            while len(self.odoms) > 10:
                self.odoms.pop(0)
            data = list(data)
            rate = 1.0 / np.mean(np.diff(self.odoms))
            data.insert(0, rate)
            # logger.info("Odometry. Rate: %0.4f Hz, x=%0.4f, y=%0.4f, t=%0.4f, vx=%0.4f, vy=%0.4f, vt=%0.4f" % tuple(data))
        elif category == "module":
            data = list(data)
            data.insert(0, recv_time)
            self.writer.writerow(data)

def main():
    if len(sys.argv) == 1:
        host = "127.0.0.1:5800"
    else:
        host = sys.argv[1]
    print("Host: %s" % host)
    address, port = host.split(":")
    port = int(port)
    interface = MyTunnel(address, port, "data")
    
    try:
        interface.start()
        while True:
            interface.update()
    finally:
        interface.stop()


if __name__ == '__main__':
    main()
