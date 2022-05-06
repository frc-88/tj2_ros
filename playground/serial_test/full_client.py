import time
import threading
import random
from tj2_tools.tunnel.serial.client import TunnelSerialClient
from tj2_tools.robot_state import Pose2d, Velocity


class MyClient(TunnelSerialClient):
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
            print(pose, velocity)
        elif result.category == "match":
            is_autonomous = result.get_int()
            match_time = result.get_double()
            team_color = result.get_string()
            print(is_autonomous, match_time, team_color)


def write_thread(tunnel):
    while True:
        tunnel.write("global", random.random(), random.random(), random.random())
        time.sleep(0.15)


def main():
    tunnel = MyClient("/dev/ttyTHS0", 115200)

    thread = threading.Thread(target=write_thread, args=(tunnel,))
    thread.daemon = True
    thread.start()

    try:
        tunnel.start()
        while True:
            tunnel.update()
            time.sleep(0.005)
    finally:
        tunnel.stop()

main()
