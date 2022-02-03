import sys
from matplotlib import pyplot as plt
import numpy as np

sys.path.insert(0, "../../tj2_tools")

from tj2_tools.particle_filter.state import FilterState
from tj2_tools.particle_filter.predictor import BouncePredictor, get_time_to_distance

predictor = BouncePredictor(
    rho=0.75,
    tau=0.05,
    g=-9.81,
    a_friction=-0.1,
    t_step=0.001,
    ground_plane=-0.1,
    a_robot=5.0,
    v_max_robot=2.0,
    t_limit=10.0
)

robot_state = FilterState()
obj_state = FilterState()
obj_state.vx = 1.0
obj_state.x = 1.0

data = []
t_intersect = get_time_to_distance(obj_state.x, 0.0, predictor.a_robot, predictor.v_max_robot, predictor.t_limit, predictor.t_step, data)

data = np.array(data)
plt.plot(data[:, 0], data[:, 1])
plt.plot(t_intersect, obj_state.x, 'x')
plt.show()
