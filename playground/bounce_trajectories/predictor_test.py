import sys
from matplotlib import pyplot as plt
import numpy as np

sys.path.insert(0, "../../tj2_tools")

from tj2_tools.particle_filter.state import FilterState
from tj2_tools.particle_filter.predictor import BouncePredictor, get_time_to_distance

from state_loader import get_states, CLASS_NAMES

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

repickle = False

# states = get_states("data/detections_2022-02-03-23-59-49.json", repickle)
states = get_states("data/detections_2022-02-04-00-02-36.json", repickle)
data = []
for state in states:
    if state.type == "odom":
        pass
        # print(state)
    elif state.type in CLASS_NAMES:
        pass
        data.append(state.stamp)
        # print(state)

data = 1.0 / np.diff(np.array(data))
data = data[np.where(data < 100)]
print(np.min(data), np.max(data))
print(np.std(data), np.mean(data))
plt.hist(data, bins=20)
plt.show()
# print("Average rage: %s" % (count / accumulator))