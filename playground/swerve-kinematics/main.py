import csv
import time
import math
from datetime import datetime

import math
import numpy as np
import matplotlib.pyplot as plt
from swerve_kinematics import SwerveKinematics
from swerve_plotter import SwervePlotter
from swerve_networktables import SwerveNetworkTables
from swerve_log_parser import SwerveLogParser

def get_config():
    # WIDTH = 10.991 / 12.0
    # LENGTH = 12.491 / 12.0
    WIDTH = 12.150 / 12.0
    LENGTH = 12.150 / 12.0

    positions = [
        [WIDTH / 2.0, LENGTH / 2.0],
        [WIDTH / 2.0, -LENGTH / 2.0],
        [-WIDTH / 2.0, -LENGTH / 2.0],
        [-WIDTH / 2.0, LENGTH / 2.0],
    ]
    num_modules = len(positions)
    swerve = SwerveKinematics(positions)
    plotter = SwervePlotter(20.0, 20.0)

    return swerve, plotter, num_modules

def live():    
    swerve, plotter, num_modules = get_config()
    swerve_nt = SwerveNetworkTables(num_modules)

    chassis_trans_speed = 1.25
    chassis_trans_angle = 0.0
    chassis_rot_speed = 0.0

    while True:
        swerve_nt.update()            
        chassis_speeds = swerve.module_to_chassis_speeds(swerve_nt.module_states.state)
        dt = swerve_nt.dt()
        if dt is None:
            continue
        if swerve_nt.x is None:
            continue
        state = swerve.estimate_pose(dt)

        plotter.append_measured_state(swerve_nt.x, swerve_nt.y)
        plotter.set_measured_arrow(swerve_nt.t)
        print("X: %0.4f\tY: %0.4f\tT:%0.4f" % (swerve_nt.x, swerve_nt.y, swerve_nt.t))

        plotter.append_calculated_state(state.x, state.y)
        plotter.set_calculated_arrow(state.t)

        plotter.pause()

        # command_motors(nt, timestamp, 1.25, 0.0, 0.0)
        # vx = chassis_trans_speed * math.cos(chassis_trans_angle)
        # vy = chassis_trans_speed * math.sin(chassis_trans_angle)
        # chassis_trans_angle += 0.15
        # if chassis_trans_angle > math.pi * 2:
        #     chassis_trans_angle = 0.0
        # command_motors(nt, timestamp, vx, vy, chassis_rot_speed)

        time.sleep(0.01)

def logged():
    path = "2021-04-24T16-50-07--353709.csv"
    # path = "2021-04-24T16-51-00--079020.csv"
    # path = "2021-04-24T16-51-32--525400.csv"
    # path = "2021-04-24T16-51-44--394209.csv"

    swerve, plotter, num_modules = get_config()
    parser = SwerveLogParser(num_modules, path)

    prev_time = None
    for timestamp, logged_state, module_state in parser.iter():
        if timestamp == 0.0:
            continue
        if prev_time is None:
            prev_time = timestamp
            input()
            continue
        
        dt = (timestamp - prev_time) * 1E-6
        prev_time = timestamp
        
        chassis_speeds = swerve.module_to_chassis_speeds(module_state.state)
        state = swerve.estimate_pose(dt)

        plotter.append_measured_state(logged_state[0], logged_state[1])
        plotter.set_measured_arrow(logged_state[2])

        plotter.append_calculated_state(state.x, state.y)
        plotter.set_calculated_arrow(state.t)
        plotter.pause()


live()
# logged()