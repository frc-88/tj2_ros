import re
import math
from module_states import ModuleStates


class SwerveLogParser:
    def __init__(self, num_modules, path):
        self.states = []
        self.logged_poses = []
        self.num_modules = num_modules
        self.line_regex = r"time: (.*), idx: (.*), wheel_speed: (.*), azimuth: (.*)"
        self.state = ModuleStates(self.num_modules)

        self.logged_poses_regex = r"2: dx: (.*), dy: (.*), dtheta: (.*)"

        self.load(path)
    
    def load(self, path):
        self.states = []
        with open(path) as file:
            contents = file.read()
        
        for line in contents.splitlines():
            self.parse_module_line(line)
            self.parse_delta_pose_line(line)
    
    def parse_module_line(self, line):
        match = re.search(self.line_regex, line)
        if not match:
            return
        
        timestamp = float(match.group(1))
        idx = int(match.group(2))
        wheel_speed = float(match.group(3))
        azimuth = float(match.group(4))

        azimuth = math.radians(azimuth)

        self.state.set(idx, wheel_speed, azimuth)
        self.states.append((timestamp, ModuleStates.from_state(self.state)))
    
    def parse_delta_pose_line(self, line):
        match = re.search(self.logged_poses_regex, line)
        if not match:
            return
        
        dx = float(match.group(1))
        dy = float(match.group(2))
        dtheta = float(match.group(3))

        self.logged_poses.append((dx, dy, dtheta))

