import time
import math
from networktables import NetworkTables

import math
import numpy as np
import matplotlib.pyplot as plt
from swerve_kinematics import SwerveKinematics
from module_states import ModuleStates
from swerve_plotter import SwervePlotter

def get_module(nt, module_num):
    wheel_speed = nt.getEntry("modules/%s/wheel" % module_num).getDouble(0.0)
    azimuth = nt.getEntry("modules/%s/azimuth" % module_num).getDouble(0.0)
    azimuth = math.radians(azimuth)
    return wheel_speed, azimuth


def command_motors(nt, timestamp, vx, vy, omega):
    nt.getEntry("command/time").setNumber(int(timestamp))
    nt.getEntry("command/linear_x").setNumber(vx)
    nt.getEntry("command/linear_y").setNumber(vy)
    nt.getEntry("command/angular_z").setNumber(omega)


def main():
    positions = [
        [-0.9159166667, 0.5204583333],
        [-0.9159166667, -0.5204583333],
        [0.9159166667, -0.5204583333],
        [0.9159166667, 0.5204583333],
    ]

    swerve = SwerveKinematics(positions)

    NetworkTables.initialize(server="10.0.88.2")
    nt = NetworkTables.getTable("Swerve")

    num_modules = 4
    module_states = ModuleStates(num_modules)

    plotter = SwervePlotter(20.0, 20.0)

    prev_time = None
    x0 = None
    y0 = None
    t0 = None

    chassis_trans_speed = 1.25
    chassis_trans_angle = 0.0
    chassis_rot_speed = 0.0

    while True:
        timestamp = nt.getEntry("odom/time").getDouble(0.0)
        x = nt.getEntry("odom/x").getDouble(0.0)
        y = nt.getEntry("odom/y").getDouble(0.0)
        t = nt.getEntry("odom/t").getDouble(0.0)
        vx = nt.getEntry("odom/vx").getDouble(0.0)
        vy = nt.getEntry("odom/vy").getDouble(0.0)
        vt = nt.getEntry("odom/vt").getDouble(0.0)

        if timestamp != 0.0:
            if x0 is None:
                x0 = x
            if y0 is None:
                y0 = y
            if t0 is None:
                t0 = t
            x -= x0
            y -= y0
            t -= t0

        if prev_time is None:
            prev_time = timestamp
            continue
        dt = (timestamp - prev_time) * 1E-6
        prev_time = timestamp

        for module_num in range(num_modules):
            wheel_speed, azimuth = get_module(nt, module_num)
            module_states.set(module_num, wheel_speed, azimuth)
            # print(wheel_speed, azimuth)
            
        chassis_speeds = swerve.module_to_chassis_speeds(module_states.state)
        state = swerve.estimate_pose(dt)

        plotter.append_measured_state(x, y)
        plotter.append_calculated_state(state.x, state.y)
        print(timestamp, state.x - x, state.y - y)
        # print(vx, vy, vt)
        plotter.pause()

        # command_motors(nt, timestamp, 1.25, 0.0, 0.0)
        vx = chassis_trans_speed * math.cos(chassis_trans_angle)
        vy = chassis_trans_speed * math.sin(chassis_trans_angle)
        chassis_trans_angle += 0.15
        if chassis_trans_angle > math.pi * 2:
            chassis_trans_angle = 0.0
        
        command_motors(nt, timestamp, vx, vy, chassis_rot_speed)

        time.sleep(0.01)

main()
