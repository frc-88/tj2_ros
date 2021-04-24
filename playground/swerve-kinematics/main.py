import csv
import time
import math
from datetime import datetime

import math
import numpy as np
import matplotlib.pyplot as plt
from swerve_kinematics import SwerveKinematics
from module_states import ModuleStates
from swerve_plotter import SwervePlotter

def main():
    

    positions = [
        [-0.9159166667, 0.5204583333],
        [-0.9159166667, -0.5204583333],
        [0.9159166667, -0.5204583333],
        [0.9159166667, 0.5204583333],
    ]

    swerve = SwerveKinematics(positions)

    num_modules = 4
    module_states = ModuleStates(num_modules)

    plotter = SwervePlotter(20.0, 20.0)


    chassis_trans_speed = 1.25
    chassis_trans_angle = 0.0
    chassis_rot_speed = 0.0

    log_row = []

    while True:
        

        log_row = [timestamp, x, y, t, vx, vy, vt]

        for module_num in range(num_modules):
            wheel_speed, azimuth = get_module(nt, module_num)
            module_states.set(module_num, wheel_speed, azimuth)
            # print(wheel_speed, azimuth)

            log_row.append(module_num)
            log_row.append(wheel_speed)
            log_row.append(azimuth)
            
        chassis_speeds = swerve.module_to_chassis_speeds(module_states.state)
        state = swerve.estimate_pose(dt)

        plotter.append_measured_state(x, y)
        plotter.append_calculated_state(state.x, state.y)
        # print(timestamp, state.x - x, state.y - y)
        # print(vx, vy, vt)
        # print(t)
        plotter.pause()

        # command_motors(nt, timestamp, 1.25, 0.0, 0.0)
        # vx = chassis_trans_speed * math.cos(chassis_trans_angle)
        # vy = chassis_trans_speed * math.sin(chassis_trans_angle)
        # chassis_trans_angle += 0.15
        # if chassis_trans_angle > math.pi * 2:
        #     chassis_trans_angle = 0.0
        
        # command_motors(nt, timestamp, vx, vy, chassis_rot_speed)

        writer.writerow(log_row)
        time.sleep(0.01)

main()
