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

states = get_states("data/detections_2022-02-03-23-59-49.json", repickle)
for state in states:
    if state.type == "odom":
        print(state)
    elif state.type in CLASS_NAMES:
        print(state)
