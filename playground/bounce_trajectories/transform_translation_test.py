import numpy as np
from tj2_tools.robot_state import Simple3DState


def method1(states, new_x, new_y, new_theta):
    origin_x = 0.0
    origin_y = 0.0
    new_states = []
    for state in states:
        cx = state.x
        cy = state.y
        delta_angle = state.theta + new_theta
        angle = delta_angle if delta_angle > 0 else delta_angle# + 2*np.pi
        cos, sin = np.cos(-angle), np.sin(-angle) # rotate in the opposite direction
        ccx, ccy = cx*cos-cy*sin, cx*sin+cy*cos
        ccx, ccy = new_x-origin_x+ccx, new_y-origin_y+ccy
        new_state = Simple3DState.from_state(state)
        new_state.x = ccx
        new_state.y = ccy
        new_state.theta = angle
        new_states.append(new_state)
    return new_states
    

def method2(states, origin_state):
    return [state.relative_to(origin_state) for state in states]
        
def test():
    states = [
        Simple3DState(1.0, 0.0, theta=0.0),
        Simple3DState(0.0, 1.0, theta=0.0),
        Simple3DState(0.0, 1.0, theta=0.5),
    ]
    origin_state = Simple3DState(0.0, 0.0, theta=1.0)
    new_states_1 = method1(states, origin_state.x, origin_state.y, origin_state.theta)
    new_states_2 = method2(states, origin_state)

    for new_state_1, new_state_2 in zip(new_states_1, new_states_2):
        print(new_state_1)
        print(new_state_2)

test()
