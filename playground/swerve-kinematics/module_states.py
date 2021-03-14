import numpy as np


class ModuleStates:
    def __init__(self, num_modules):
        self.num_modules = num_modules
        self.state = np.zeros((num_modules, 2))
    
    def set(self, index, wheel_speed, azimuth):
        self.state[index][0] = azimuth
        self.state[index][1] = wheel_speed
    
    @classmethod
    def from_state(cls, state):
        self = cls(state.num_modules)
        self.state = np.copy(state.state)
        return self
