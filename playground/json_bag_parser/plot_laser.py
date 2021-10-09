import sys
import math
import json
import time
import numpy as np
from matplotlib import pyplot as plt


def ranges_to_dots(message):
    ranges = message["ranges"]
    angle_min = message["angle_min"]
    # angle_max = message["angle_max"]
    angle_increment = message["angle_increment"]
    
    # assert (len(ranges) - 1) * angle_increment + angle_min <= angle_max
    dots = []
    for index, distance in enumerate(ranges):
        if np.isinf(distance):
            continue
        angle = angle_increment * index + angle_min
        x = distance * math.cos(angle)
        y = distance * math.sin(angle)

        dots.append((x, y))
    return np.array(dots)

def main():
    print("Press Q in the plot window then ^C in the terminal window to exit")
    path = sys.argv[1]

    simulate_real_time = False
    set_window_to_laser_max = False

    plt.ion()
    fig = plt.figure(constrained_layout=True, figsize=(10.0, 9.0))
    ax = fig.add_subplot()
    # fig.show()

    plot_data = ax.plot([0, 1], [0, 1], marker='.', linestyle="")[0]
    ax.plot([0], [0], marker='x', linestyle='', color='k')
    x_limits = None
    y_limits = None

    sim_start_time = 0.0
    real_start_time = time.time()

    try:
        with open(path) as file:
            bag = json.load(file)
            for timestamp, topic, message in bag:
                if topic != "/laser/scan":
                    continue
                if sim_start_time == 0.0:
                    sim_start_time = timestamp
                    real_start_time = time.time()
                sim_time = timestamp - sim_start_time
                real_time = time.time() - real_start_time

                if x_limits is None or y_limits is None:
                    if set_window_to_laser_max:
                        range_max = message["range_max"]
                        x_limits = [-range_max, range_max]
                        y_limits = [-range_max, range_max]
                    else:
                        x_limits = [0.0, 0.0]
                        y_limits = [0.0, 0.0]
                dots = ranges_to_dots(message)
                plot_data.set_data(dots[:, 0], dots[:, 1])
                msg_limits = np.array([np.min(dots, axis=0), np.max(dots, axis=0)])
            
                x_limits[0] = min(x_limits[0], msg_limits[0][0])
                x_limits[1] = max(x_limits[1], msg_limits[1][0])
                y_limits[0] = min(y_limits[0], msg_limits[0][1])
                y_limits[1] = max(y_limits[1], msg_limits[1][1])
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
