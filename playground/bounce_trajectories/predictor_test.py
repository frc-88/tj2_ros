import sys
import time
from matplotlib import pyplot as plt
import numpy as np

sys.path.insert(0, "../../tj2_tools")

from tj2_tools.particle_filter.state import FilterState, DeltaMeasurement
from tj2_tools.predictions.predictor import BouncePredictor, get_time_to_distance

from state_loader import get_states, CLASS_NAMES
from plotter import LivePlotter2D


def main():
    predictor = BouncePredictor(
        rho=0.75,
        tau=0.05,
        g=-9.81,
        a_friction=-0.1,
        t_step=0.001,
        ground_plane=-0.05,
        a_robot=5.0,
        v_max_robot=2.0,
        t_limit=10.0
    )
    delta_meas = DeltaMeasurement()

    x_width = 20.0
    y_width = 20.0
    z_width = 3.0
    plotter = LivePlotter2D(x_width, y_width)

    repickle = False

    # path = "data/prediction_data_2/detections_2022-02-09-10-43-43.json"
    # path = "data/prediction_data_2/detections_2022-02-09-10-56-07.json"
    # path = "data/prediction_data_2/detections_2022-02-09-10-44-36.json"
    # path = "data/prediction_data_2/detections_2022-02-09-10-45-09.json"
    # path = "data/prediction_data_2/detections_2022-02-09-10-45-41.json"
    # path = "data/prediction_data_2/detections_2022-02-09-10-46-11.json"
    # path = "data/prediction_data_2/detections_2022-02-09-10-54-38.json"
    path = "data/prediction_data_2/detections_2022-02-09-10-55-28.json"
    # path = "data/prediction_data_2/detections_2022-02-09-10-56-07.json"
    # path = "data/prediction_data_2/detections_2022-02-09-10-57-39.json"
    # path = "data/prediction_data_2/detections_2022-02-09-10-58-15.json"
    # path = "data/prediction_data_2/detections_2022-02-09-11-00-08.json"

    states = get_states(path, repickle)
    odom_state = FilterState()
    object_state = FilterState()
    predicted_state = FilterState()

    sim_start_t = states[0].stamp
    real_start_t = time.time()

    try:
        for state in states:
            current_time = time.time()
            pause_start_t = current_time
            while plotter.is_paused:
                plotter.pause()
                current_time = time.time()
            real_start_t += current_time - pause_start_t

            sim_time = state.stamp
            real_time = current_time
            sim_duration = sim_time - sim_start_t
            real_duration = real_time - real_start_t

            if state.type in CLASS_NAMES:
                object_state = state.relative_to(odom_state)
                object_state = delta_meas.update(object_state)

                # predicted_state = predictor.get_prediction(object_state, 1.0)
                predicted_state = predictor.get_robot_intersection(odom_state, object_state)
                predicted_state.stamp -= sim_start_t
            elif state.type == "odom":
                odom_state = state

            if sim_duration >= real_duration:  # if simulation time has caught up to real time, spend some time drawing
                plotter.draw(sim_duration, object=object_state, odom=odom_state, prediction=predicted_state)
                plotter.pause()
    finally:
        plotter.stop()


if __name__ == '__main__':
    main()
