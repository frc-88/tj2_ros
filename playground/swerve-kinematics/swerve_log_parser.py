import csv
import math
from module_states import ModuleStates


class SwerveLogParser:
    def __init__(self, num_modules, path):
        self.states = []
        self.logged_poses = []
        self.num_modules = num_modules
        self.state = ModuleStates(self.num_modules)

        self.load(path)
    
    def load(self, path):
        self.states = []
        with open(path) as file:
            reader = csv.reader(file)

            for line in reader:
                timestamp = line[0]
                x = line[1]
                y = line[2]
                theta = line[3]
                vx = line[4]
                vy = line[5]
                vt = line[6]
                
                self.logged_poses.append((x, y, theta, vx, vy, vt))

                for index in range(7, len(line), 3):
                    module_idx = line[index]
                    wheel_speed = line[index + 1]
                    azimuth = line[index + 2]
                    self.state.set(module_idx, wheel_speed, azimuth)
                    self.states.append((timestamp, ModuleStates.from_state(self.state)))
