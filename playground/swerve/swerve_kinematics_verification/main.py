import csv
import time
import math
from datetime import datetime

import math
import numpy as np
import matplotlib.pyplot as plt
from swerve_kinematics import SwerveKinematics
from swerve_plotter import SwervePlotter
from module_plotter import ModulePlotter
from swerve_networktables import SwerveNetworkTables
from swerve_log_parser import SwerveLogParser

def get_config():
    # WIDTH = 10.991 / 12.0
    # LENGTH = 12.491 / 12.0
    WIDTH = 12.150 / 12.0
    LENGTH = 12.150 / 12.0

    positions = [
        [WIDTH / 2.0, LENGTH / 2.0],
        [-WIDTH / 2.0, LENGTH / 2.0],
        [-WIDTH / 2.0, -LENGTH / 2.0],
        [WIDTH / 2.0, -LENGTH / 2.0],
    ]
    num_modules = len(positions)
    swerve = SwerveKinematics(positions)
    swerve_plotter = SwervePlotter(20.0, 20.0)
    module_plotter = ModulePlotter(positions, 4.0, 4.0)

    return swerve, swerve_plotter, module_plotter, num_modules

def live():    
    swerve, swerve_plotter, module_plotter, num_modules = get_config()
    swerve_nt = SwerveNetworkTables(num_modules)

    chassis_trans_speed = 0.5
    chassis_trans_angle = 0.0
    chassis_rot_speed = 0.0

    enable_swerve_plotter = True
    # enable_swerve_plotter = False
    enable_module_plotter = True
    # enable_module_plotter = False

    prev_cmd_time = time.time()

    while True:
        swerve_nt.update()
        chassis_speeds = swerve.module_to_chassis_speeds(swerve_nt.module_states.state)
        dt = swerve_nt.dt()
        if dt is None:
            continue
        if swerve_nt.x is None:
            continue
        state = swerve.estimate_pose(dt)

        if enable_swerve_plotter:
            swerve_plotter.append_measured_state(swerve_nt.x, swerve_nt.y)
            swerve_plotter.set_measured_arrow(swerve_nt.t)

            swerve_plotter.append_calculated_state(state.x, state.y)
            swerve_plotter.set_calculated_arrow(state.t)

            swerve_plotter.pause()
        if enable_module_plotter:
            for module_num in range(swerve_nt.num_modules):
                azimuth, wheel_speed = swerve_nt.module_states.get(module_num)
                module_plotter.set_module_arrow(module_num, wheel_speed, azimuth)
            #     print("%0.4f\t%0.4f" % (wheel_speed, azimuth), end="\t|\t")
            # print()
            module_plotter.pause()
        # print("X: %0.4f\tY: %0.4f\tT:%0.4f" % (swerve_nt.x, swerve_nt.y, swerve_nt.t))
        # print("X: %0.4f\tY: %0.4f\tT:%0.4f" % (state.x, state.y, state.t))
        print("X: %0.4f\tY: %0.4f\tT:%0.4f" % (state.vx, state.vy, state.vt))
        # for module_num in range(swerve_nt.num_modules):
        #     azimuth, wheel_speed = swerve_nt.module_states.get(module_num)
        #     print("%0.4f\t%0.4f" % (wheel_speed, azimuth), end="\t|\t")
        # print()

        # chassis_trans_angle += 1.0
        # if chassis_trans_angle > 360.0:
        #     chassis_trans_angle = 0.0
        # swerve_nt.command_motors(chassis_trans_speed, chassis_trans_angle, chassis_rot_speed)
        if (time.time() - prev_cmd_time > 3.0):
            swerve_nt.command_motors(0.0, 0.0, 5.0)
            prev_cmd_time = time.time()

        time.sleep(0.01)

def logged():
    path = "data/2021-10-02T18-49-40--532382.csv"

    swerve, swerve_plotter, module_plotter, num_modules = get_config()
    parser = SwerveLogParser(num_modules, path)

    prev_time = None
    for timestamp, logged_state, module_state in parser.iter():
        if timestamp == 0.0:
            continue
        if prev_time is None:
            prev_time = timestamp
            # input()
            continue
        
        dt = (timestamp - prev_time) * 1E-6
        prev_time = timestamp
        
        chassis_speeds = swerve.module_to_chassis_speeds(module_state.state)
        state = swerve.estimate_pose(dt)

        swerve_plotter.append_measured_state(logged_state[0], logged_state[1])
        swerve_plotter.set_measured_arrow(logged_state[2])

        swerve_plotter.append_calculated_state(state.x, state.y)
        swerve_plotter.set_calculated_arrow(state.t)
        swerve_plotter.pause()

        print(module_state.state)
        for module_num in range(num_modules):
            azimuth, wheel_speed = module_state.state[module_num]
            module_plotter.set_module_arrow(module_num, wheel_speed, azimuth)
            module_plotter.pause()


# live()
logged()