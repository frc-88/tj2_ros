
from os import confstr
import sys
import math
import json
import time
import numpy as np
import threading
from queue import Queue
from matplotlib import pyplot as plt
from matplotlib.patches import Arrow
from matplotlib.patches import Arc
from matplotlib.patches import RegularPolygon
from scipy.spatial.transform import Rotation


def quat_to_yaw(quaternion):
    return Rotation.from_quat([
        quaternion["x"],
        quaternion["y"],
        quaternion["z"],
        quaternion["w"]
    ]).as_euler('xyz')[2]


def bag_task(thread_state, state, lock, path):
    thread_state["is_running"] = True
    module_topic = "/tj2/swerve_modules/"
    command_topic = "/tj2/cmd_vel"

    simulate_real_time = True

    sim_start_time = 0.0
    real_start_time = time.time()
    
    try:
        with open(path) as file:
            bag = json.load(file)
            for timestamp, topic, message in bag:
                if thread_state["should_stop"]:
                    print("Bag task is stopping")
                    break
                is_module_topic = topic.startswith(module_topic)
                if not (is_module_topic or topic == command_topic):
                    continue

                if sim_start_time == 0.0:
                    sim_start_time = timestamp
                    real_start_time = time.time()
                sim_time = timestamp - sim_start_time
                real_time = time.time() - real_start_time

                if simulate_real_time and sim_time - real_time < 0.0:
                    continue
                
                with lock:
                    if topic == command_topic:
                        state["command"] = message
                    elif is_module_topic:
                        module_index = int(topic[len(module_topic):])
                        state["modules"][module_index] = message

                if simulate_real_time and sim_time - real_time > 0.0:
                    time.sleep(sim_time - real_time)
                
    finally:
        thread_state["is_running"] = False

class CircularArrow:
    def __init__(self, ax, radius, centX, centY, angle, theta2, color='black', lw=5):
        # Draw arc
        self.arc = Arc([centX, centY], radius, radius, angle=min(angle, theta2),
            theta1=0, theta2=max(angle, theta2), capstyle='round', linestyle='-', lw=lw, color=color)
        ax.add_patch(self.arc)

        # Create the arrow head
        if angle > theta2:
            angle = theta2
            theta2 = 180.0
            positional_angle = math.radians(angle)
        else:
            positional_angle = math.radians(theta2 + angle)
        endX = centX + (radius / 2) * np.cos(positional_angle) # Do trig to determine end position
        endY = centY + (radius / 2) * np.sin(positional_angle)

        # Create triangle as arrow head
        self.arrow_head = RegularPolygon(
            (endX, endY),            # (x,y)
            3,                       # number of vertices
            radius / 6,                # radius
            math.radians(angle + theta2),     # orientation
            color=color
        )
        ax.add_patch(self.arrow_head)
    
    def remove(self):
        self.arc.remove()
        self.arrow_head.remove()


def main():
    print("Press Q in the plot window then ^C in the terminal window to exit")
    path = sys.argv[1]
    
    state = {
        "modules": {},
        "command": None
    }

    thread_state = dict(
        should_stop=False,
        is_running=False
    )

    lock = threading.Lock()

    thread = threading.Thread(target=bag_task, args=(thread_state, state, lock, path))
    thread.start()
    while not thread_state["is_running"]:
        pass

    module_arrows = {}
    linear_command_arrow = None
    angular_command_arrow = None
    # prev_data = None

    plt.ion()
    fig = plt.figure(constrained_layout=True, figsize=(10.0, 10.0))
    figure_num = plt.gcf().number
    ax = fig.add_subplot()
    # fig.show()


    ax.plot([0.0], [0.0], marker='.', linestyle="")[0]

    max_linear_vel = 4.48
    max_angular_vel = 61.4
    window_size = 0.5

    ax.set_xlim([-window_size, window_size])
    ax.set_ylim([-window_size, window_size])

    WIDTH = 0.30861
    LENGTH = 0.30861
    module_locations = (
        (WIDTH / 2.0, LENGTH / 2.0),
        (-WIDTH / 2.0, LENGTH / 2.0),
        (-WIDTH / 2.0, -LENGTH / 2.0),
        (WIDTH / 2.0, -LENGTH / 2.0),
    )

    try:
        while thread_state["is_running"]:
            if not plt.fignum_exists(figure_num):
                print("Plot closed")
                break
            with lock:
                for state_type, state_data in state.items():
                    if state_type == "modules":
                        for module_index, message in state_data.items():
                            azimuth_position = message["azimuth_position"]
                            wheel_velocity = message["wheel_velocity"]
                            location_x, location_y = module_locations[module_index]

                            if module_index not in module_arrows:
                                arrow = Arrow(0.0, 0.0, 1.0, 1.0, color='k')
                                ax.add_patch(arrow)
                                module_arrows[module_index] = arrow
                            else:
                                arrow = module_arrows[module_index]

                            arrow_len = wheel_velocity * 0.1 + 0.01
                            arrow_x = arrow_len * math.cos(azimuth_position)
                            arrow_y = arrow_len * math.sin(azimuth_position)
                            # curr_data = location_x + arrow_x, location_y + arrow_y
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
                    elif state_type == "command":
                        if state_data is None:
                            continue
                        if linear_command_arrow is None:
                            linear_command_arrow = Arrow(0.0, 0.0, 1.0, 1.0, color='k')
                            ax.add_patch(linear_command_arrow)
                        
                        arrow_x = state_data["linear"]["x"] * window_size / max_linear_vel
                        arrow_y = state_data["linear"]["y"] * window_size / max_linear_vel

                        linear_command_arrow.remove()
                        linear_command_arrow = Arrow(0.0, 0.0, arrow_x, arrow_y, color='k', width=0.05)
                        ax.add_patch(linear_command_arrow)

                        if angular_command_arrow is None:
                            angular_command_arrow = CircularArrow(ax, 0.0, 0.0, 0.0, 0.0, 0.0)
                        
                        scaled_ang_v = state_data["angular"]["z"] * window_size / max_angular_vel
                        angular_command_arrow.remove()
                        if scaled_ang_v >= 0.0:
                            start_angle = 0.0
                            stop_angle = 340.0
                        else:
                            start_angle = 340.0
                            stop_angle = 0.0
                        angular_command_arrow = CircularArrow(ax, abs(scaled_ang_v) + 0.01, 0.0, 0.0, start_angle, stop_angle)

            plt.pause(0.001)
            
    finally:
        print("Main thread has stopped running")
        thread_state["should_stop"] = True
        plt.ioff()

if __name__ == "__main__":
    main()
