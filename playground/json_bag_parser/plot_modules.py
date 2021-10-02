
from os import confstr
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


def main():
    print("Press Q in the plot window then ^C in the terminal window to exit")
    path = sys.argv[1]

    simulate_real_time = True

    plt.ion()
    fig = plt.figure(constrained_layout=True, figsize=(10.0, 10.0))
    ax = fig.add_subplot()
    # fig.show()

    ax.plot([0.0], [0.0], marker='.', linestyle="")[0]

    sim_start_time = 0.0
    real_start_time = time.time()

    module_arrows = {}
    
    ax.set_xlim([-0.5, 0.5])
    ax.set_ylim([-0.5, 0.5])

    module_topic = "/tj2/swerve_modules/"
    # prev_data = None

    try:
        with open(path) as file:
            bag = json.load(file)
            for timestamp, topic, message in bag:
                if not topic.startswith(module_topic):
                    continue

                if sim_start_time == 0.0:
                    sim_start_time = timestamp
                    real_start_time = time.time()
                sim_time = timestamp - sim_start_time
                real_time = time.time() - real_start_time

                if simulate_real_time and sim_time - real_time < 0.0:
                    continue
                
                module_index = topic[len(module_topic):]
                if module_index not in module_arrows:
                    arrow = Arrow(0.0, 0.0, 1.0, 1.0, color='k')
                    ax.add_patch(arrow)
                    module_arrows[module_index] = arrow
                else:
                    arrow = module_arrows[module_index]
                
                azimuth_position = message["azimuth_position"]
                wheel_velocity = message["wheel_velocity"]
                location_x = message["location_x"]
                location_y = message["location_y"]

                arrow_len = wheel_velocity * 0.1
                arrow_x = arrow_len * math.cos(azimuth_position)
                arrow_y = arrow_len * math.sin(azimuth_position)
                curr_data = location_x + arrow_x, location_y + arrow_y
                # if prev_data is None:
                #     prev_data = curr_data
                # if abs(curr_data[0] - prev_data[0]) < 0.4 and abs(curr_data[1] - prev_data[1]) < 0.4:
                #     continue
                # print(curr_data, prev_data)
                # prev_data = curr_data

                arrow.remove()
                arrow = Arrow(location_x, location_y, arrow_x, arrow_y, color='k', width=0.05)
                module_arrows[module_index] = arrow
                ax.add_patch(arrow)
                print(sim_time - real_time)

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
