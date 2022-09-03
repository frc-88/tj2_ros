import csv
import math
from module_states import ModuleStates


class SwerveLogParser:
    def __init__(self, num_modules, path):
        self.data = []
        self.num_modules = num_modules
        self.state = ModuleStates(self.num_modules)

        self.load(path)
    
    def load(self, path):
        self.data = []
        with open(path) as file:
            reader = csv.reader(file)

            for line in reader:
                timestamp = float(line[0])
                x = float(line[1])
                y = float(line[2])
                theta = float(line[3])
                vx = float(line[4])
                vy = float(line[5])
                vt = float(line[6])

                for index in range(7, len(line), 3):
                    module_idx = int(line[index])
                    wheel_speed = float(line[index + 1])
                    azimuth = float(line[index + 2])

                    self.state.set(module_idx, azimuth, wheel_speed)

                self.data.append((timestamp, (x, y, theta, vx, vy, vt), ModuleStates.from_state(self.state)))
        
    def iter(self):
        for index in range(len(self.data)):
            yield self.data[index]

