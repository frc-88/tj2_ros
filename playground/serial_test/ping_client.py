import sys
import time
import numpy as np
import random
import threading

sys.path.insert(0, "../../tj2_tools")

from tj2_tools.tunnel.serial.client import TunnelSerialClient


class MyClient(TunnelSerialClient):
    def __init__(self, address, baud):
        super().__init__(address, baud)
        self.pings = []

    def packet_callback(self, result):
        if result.category == "ping":
            ping = result.recv_time - result.get_double()
            print("Ping: %0.5f" % (ping))
            self.pings.append(ping)


def main():
    tunnel = MyClient("/dev/ttyTHS0", 115200)

    last_ping = time.time()
    try:
        while True:
            if time.time() - last_ping > 0.05:
                tunnel.write("ping", time.time())
                last_ping = time.time()
                
            tunnel.update()
    finally:
        tunnel.stop()
        print("mean:", np.mean(tunnel.pings))
        print("stddev:", np.std(tunnel.pings))
        print("len:", len(tunnel.pings))

main()
