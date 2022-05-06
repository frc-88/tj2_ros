import sys
import time
from matplotlib import pyplot as plt
import numpy as np

sys.path.insert(0, "../../tj2_tools")

from tj2_tools.robot_state import Simple3DState
from tj2_tools.predictions.predictor import BouncePredictor

from state_loader import get_states, CLASS_NAMES
from plotter import LivePlotter2D


def main():
    predictor = BouncePredictor(
        v_max_robot=4.0,
        past_window_size=4,
        vx_std_dev_threshold=1.0,
        vy_std_dev_threshold=1.0
    )

    x_width = 40.0
    y_width = 40.0
    z_width = 3.0
    plotter = LivePlotter2D(x_width, y_width)

    repickle = True

    # detections_2022-02-09-10-43-43.json <-> video-2022-02-09T10-43-45--876036.json
    # detections_2022-02-09-10-44-36.json <-> video-2022-02-09T10-44-38--000074.json
    # detections_2022-02-09-10-45-09.json <-> video-2022-02-09T10-45-11--254047.json
    # detections_2022-02-09-10-45-41.json <-> video-2022-02-09T10-45-43--102240.json
    # detections_2022-02-09-10-46-11.json <-> video-2022-02-09T10-46-13--575758.json
    # detections_2022-02-09-10-54-38.json <-> video-2022-02-09T10-54-41--501451.json
    # detections_2022-02-09-10-55-28.json <-> video-2022-02-09T10-55-31--215422.json
    # detections_2022-02-09-10-56-07.json <-> video-2022-02-09T10-56-09--777772.json
    # detections_2022-02-09-10-57-39.json <-> video-2022-02-09T10-57-41--850998.json
    # detections_2022-02-09-10-58-15.json <-> video-2022-02-09T10-58-17--471241.json
    # detections_2022-02-09-11-00-08.json <-> video-2022-02-09T11-00-11--029853.json

    # path = "data/detections_2022-02-03-23-59-49.json"
    # path = "data/detections_2022-02-04-00-02-36.json"
    # path = "data/prediction_data_2/detections_2022-02-09-10-43-43.json"  # stationary object, moving robot
    path = "data/prediction_data_2/detections_2022-02-09-10-44-36.json"  # stationary robot, object from behind left
    # path = "data/prediction_data_2/detections_2022-02-09-10-45-09.json"  # stationary robot, object from behind right
    # path = "data/prediction_data_2/detections_2022-02-09-10-45-41.json"  # stationary robot, object from left
    # path = "data/prediction_data_2/detections_2022-02-09-10-46-11.json"  # stationary robot, object from right
    # path = "data/prediction_data_2/detections_2022-02-09-10-54-38.json"# robot gets confused
    # path = "data/prediction_data_2/detections_2022-02-09-10-55-28.json"  # robot chases down object 1
    # path = "data/prediction_data_2/detections_2022-02-09-10-56-07.json"  # robot chases down object 2
    # path = "data/prediction_data_2/detections_2022-02-09-10-57-39.json"  # robot chases down object 3
    # path = "data/prediction_data_2/detections_2022-02-09-10-58-15.json"  # robot chases down object 4
    # path = "data/prediction_data_2/detections_2022-02-09-11-00-08.json"  # robot chases down object 5

    states = get_states(path, repickle)
    odom_state = Simple3DState()
    object_state = Simple3DState()
    predicted_state = Simple3DState()

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

            if state.stamp == 0.0:
                print(state)
                continue

            sim_time = state.stamp
            real_time = current_time
            sim_duration = sim_time - sim_start_t
            real_duration = real_time - real_start_t

            if state.type in CLASS_NAMES:
                object_state = state.relative_to(odom_state)
                predicted_state = predictor.get_robot_intersection(object_state, odom_state)
                if predicted_state is not None:
                    predicted_state.stamp -= sim_start_t
                    # predicted_state = predicted_state.relative_to(odom_state)
            elif state.type == "odom":
                odom_state = state

            if sim_duration >= real_duration:  # if simulation time has caught up to real time, spend some time drawing
                plotter.draw(sim_duration, object=object_state, odom=odom_state, prediction=predicted_state)
                plotter.pause()
    finally:
        plotter.stop()


if __name__ == '__main__':
    main()
