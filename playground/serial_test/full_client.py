import sys
import time
import numpy as np
import random
import threading

sys.path.insert(0, "../../tj2_tools")

from tj2_tools.tunnel.serial.client import TunnelSerialClient
from tj2_tools.robot_state import Pose2d, Velocity


class MyClient(TunnelSerialClient):
    def __init__(self, address, baud):
        super().__init__(address, baud)
        self.pings = []

    def packet_callback(self, result):
        if result.category == "odom":
            x = result.get_double()
            y = result.get_double()
            theta = result.get_double()
            vx = result.get_double()
            vy = result.get_double()
            vt = result.get_double()
            pose = Pose2d(x, y, theta)
            velocity = Velocity(vx, vy, vt)
            # print(pose, velocity)
        elif result.category == "match":
            is_autonomous = result.get_int()
            match_time = result.get_double()
            team_color = result.get_string()
            # print(is_autonomous, match_time, team_color)
        elif result.category == "ping":
            ping = result.recv_time - result.get_double()
            print("Ping: %0.5f" % (ping))
            self.pings.append(ping)
        elif result.category == "joint":
            index = result.get_int()
            position = result.get_double()
            # print("Joint: %s -> %0.4f" % (index, position))


def write_thread(tunnel):
    last_ping = time.time()
    while True:
        tunnel.write("global", random.random(), random.random(), random.random())
        time.sleep(1.0 / 30.0)
        if time.time() - last_ping > 0.05:
            tunnel.write("ping", time.time())
            last_ping = time.time()


def main():
    tunnel = MyClient("/dev/ttyTHS0", 115200)

    thread = threading.Thread(target=write_thread, args=(tunnel,))
    thread.daemon = True
    thread.start()

    try:
        while True:
            tunnel.update()
            # time.sleep(0.005)
    finally:
        tunnel.stop()
        print("mean:", np.mean(tunnel.pings))
        print("stddev:", np.std(tunnel.pings))
        print("len:", len(tunnel.pings))

main()
