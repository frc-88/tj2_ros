
import sys
import math
import json
import time
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.patches import Arrow
from scipy.spatial.transform import Rotation


def quat_to_yaw(quaternion):
    return Rotation.from_quat([
        quaternion["x"],
        quaternion["y"],
        quaternion["z"],
        quaternion["w"]
    ]).as_euler('xyz')[2]

def odom_to_coordinate(message):
    position = message["pose"]["pose"]["position"]
    orientation = message["pose"]["pose"]["orientation"]

    return position["x"], position["y"], quat_to_yaw(orientation)

def main():
    print("Press Q in the plot window then ^C in the terminal window to exit")
    path = sys.argv[1]

    simulate_real_time = False

    plt.ion()
    fig = plt.figure(constrained_layout=True, figsize=(10.0, 10.0))
    ax = fig.add_subplot()
    # fig.show()

    plot_data = ax.plot([0.0], [0.0], marker='.', linestyle="")[0]
    odom_x_path = []
    odom_y_path = []
    x_limits = None
    y_limits = None

    sim_start_time = 0.0
    real_start_time = time.time()

    arrow = Arrow(0.0, 0.0, 1.0, 1.0, color='k')  #head_width=0.25, head_length=0.25, fc='gray', ec='gray')
    ax.add_patch(arrow)

    try:
        with open(path) as file:
            bag = json.load(file)
            for timestamp, topic, message in bag:
                if topic != "/tj2/odom":
                    continue
                    
                if sim_start_time == 0.0:
                    sim_start_time = timestamp
                    real_start_time = time.time()
                sim_time = timestamp - sim_start_time
                real_time = time.time() - real_start_time

                if x_limits is None or y_limits is None:
                    x_limits = [0.0, 0.0]
                    y_limits = [0.0, 0.0]
                
                x, y, theta = odom_to_coordinate(message)
                odom_x_path.append(x)
                odom_y_path.append(y)

                plot_data.set_data(odom_x_path, odom_y_path)

                arrow_len = 0.1
                arrow_x = arrow_len * math.cos(theta)
                arrow_y = arrow_len * math.sin(theta)
                arrow.remove()
                arrow = Arrow(x, y, arrow_x, arrow_y, color='k', width=0.05)
                ax.add_patch(arrow)

                x_limits[0] = min(x_limits[0], min(odom_x_path), x + arrow_x)
                x_limits[1] = max(x_limits[1], max(odom_x_path), x + arrow_x)
                y_limits[0] = min(y_limits[0], min(odom_y_path), y + arrow_y)
                y_limits[1] = max(y_limits[1], max(odom_y_path), y + arrow_y)

                ax.set_xlim(x_limits)
                ax.set_ylim(y_limits)
                if simulate_real_time and sim_time - real_time > 0.0:
                    plt.pause(sim_time - real_time)
                    time.sleep(sim_time - real_time)
                else:
                    plt.pause(0.001)

    finally:
        plt.ioff()
        fig.show()

if __name__ == "__main__":
    main()
