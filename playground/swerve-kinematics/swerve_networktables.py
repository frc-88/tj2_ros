import csv
import math
import time
from networktables import NetworkTables
from datetime import datetime

from module_states import ModuleStates


class SwerveNetworkTables:
    def __init__(self, num_modules):        
        NetworkTables.initialize(server="10.0.88.2")
        self.nt = NetworkTables.getTable("swerveLibrary")

        self.num_modules = num_modules
        self.module_states = ModuleStates(self.num_modules)

        self.prev_time = None
        self.x0 = None
        self.y0 = None
        self.t0 = None

        self.timestamp = None
        self.x = None
        self.y = None
        self.t = None
        self.vx = None
        self.vy = None
        self.vt = None

        self.writer = None
        self.log_file = None

        self.start_log()

    def update(self):
        self.timestamp = self.nt.getEntry("timestamp").getDouble(0.0)
        if self.timestamp == 0.0:
            return

        self.x = self.nt.getEntry("odometryState/xPosition").getDouble(0.0)
        self.y = self.nt.getEntry("odometryState/yPosition").getDouble(0.0)
        self.t = math.radians(self.nt.getEntry("odometryState/theta").getDouble(0.0))
        self.vx = self.nt.getEntry("odometryState/xVelocity").getDouble(0.0)
        self.vy = self.nt.getEntry("odometryState/yVelocity").getDouble(0.0)
        self.vt = math.radians(self.nt.getEntry("odometryState/thetaVelocity").getDouble(0.0))

        if self.x0 is None:
            self.x0 = self.x
        if self.y0 is None:
            self.y0 = self.y
        if self.t0 is None:
            self.t0 = self.t
        self.x -= self.x0
        self.y -= self.y0
        self.t -= self.t0
        
        for module_num in range(self.num_modules):
            azimuth, wheel_speed = self.get_module(module_num)
            self.module_states.set(module_num, azimuth, wheel_speed)
        
        self.log()
    
    def dt(self):
        if self.timestamp is None:
            return None
        else:
            if self.prev_time is None:
                self.prev_time = self.timestamp
                return None
            dt = (self.timestamp - self.prev_time) * 1E-6
            self.prev_time = self.timestamp
            return dt

    def start_log(self):
        now = datetime.now()
        filename = now.strftime("%Y-%m-%dT%H-%M-%S--%f.csv")
        self.log_file = open("data/" + filename, 'w', newline='')
        self.writer = csv.writer(self.log_file)

    def log(self):
        if self.writer is None:
            return
        log_row = [self.timestamp, self.x, self.y, self.t, self.vx, self.vy, self.vt]
        for module_num in range(self.num_modules):
            azimuth, wheel_speed = self.module_states.get(module_num)
            log_row.append(module_num)
            log_row.append(wheel_speed)
            log_row.append(azimuth)

        self.writer.writerow(log_row)
        
    def get_module(self, module_num):
        wheel_speed = self.nt.getEntry("modules/%s/wheelVelocity" % module_num).getDouble(0.0)
        azimuth = self.nt.getEntry("modules/%s/azimuthPosition" % module_num).getDouble(0.0)
        azimuth = math.radians(azimuth)
        return azimuth, wheel_speed


    def command_motors(self, speed, direction, omega):
        self.nt.getEntry("commands/timestamp").setNumber(time.time())
        self.nt.getEntry("commands/translationSpeed").setNumber(speed)
        self.nt.getEntry("commands/translationDirection").setNumber(direction)
        self.nt.getEntry("commands/rotationVelocity").setNumber(omega)

