from tj2_tools.robot_state import VelocityFilter
from tj2_tools.robot_state import DeltaTimer
from tj2_tools.robot_state import Simple3DState


class DeltaMeasurement:
    def __init__(self, k=None):
        smooth_k = k
        self.vx_filter = VelocityFilter(smooth_k)
        self.vy_filter = VelocityFilter(smooth_k)
        self.vz_filter = VelocityFilter(smooth_k)
        self.timer = DeltaTimer()
        self.state = Simple3DState()
    
    def set_smooth_k(self, smooth_k):
        self.vx_filter.k = smooth_k
        self.vy_filter.k = smooth_k
        self.vz_filter.k = smooth_k

    def update(self, state: Simple3DState):
        dt = self.timer.dt(state.stamp)
        if dt == 0.0:
            self.state = state
            return state
        new_state = Simple3DState.from_state(state)
        new_state.vx = self.vx_filter.update(dt, state.x)
        new_state.vy = self.vy_filter.update(dt, state.y)
        new_state.vz = self.vz_filter.update(dt, state.z)
        self.state = new_state
        return new_state

