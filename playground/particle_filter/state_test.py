import sys

sys.path.insert(0, "../../tj2_tools")

import math
from tj2_tools.particle_filter.state import FilterState

state = FilterState()
state.x = 10.0

odom = FilterState()
odom.theta = math.pi / 2
odom.x = -10.0

state_odom = state.relative_to(odom)

state_back = odom.relative_to(-state_odom)

print("original:", state)
print("odom:", odom)
print("state in odom:", state_odom)
print("back to original:", state_back)
print(abs(state - state_back) < 1E-6)
