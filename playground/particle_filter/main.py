import sys

sys.path.insert(0, "../../tj2_tools")

import time
import numpy as np
from tj2_tools.particle_filter import JitParticleFilter as ParticleFilter
# from tj2_tools.particle_filter import ParticleFilter
from tj2_tools.particle_filter import FilterSerial
from tj2_tools.particle_filter.state import *
import state_loader
from state_loader import OBJECT_NAMES
from plotter import ParticleFilterPlotter3D, ParticleFilterPlotter2D, ParticleFilterPredictionPlotter2D
from tj2_tools.particle_filter.predictor import BouncePredictor


def main():
    repickle = False
    # path = "./data/detections_2022-01-14-12-02-32.json"  # stationary object with moving robot
    # path = "./data/detections_2022-01-14-12-39-34.json"  # short run drop
    # path = "./data/detections_2022-01-14-13-42-43.json"  # circling in YZ. Drop at the end
    # path = "./data/detections_2022-01-14-14-05-17.json"  # small motions from robot and target
    path = "./data/detections_2022-01-14-18-22-38.json"  # rapid rotations in robot theta
    # path = "./data/detections_2022-01-16-23-24-58.json"  # roll
    # path = "./data/detections_2022-01-16-23-24-58.json"  # bounce

    states = state_loader.get_states(path, repickle)

    meas_std_val = 0.05
    u_std = [0.02, 0.02, 0.1, 0.02]
    initial_range = [1.0, 1.0, 1.0, 0.25, 0.25, 0.25]
    stale_filter_time = 0.1
    num_particles = 250
    bounds = [
        [-20.0, 20.0],
        [-20.0, 20.0],
        [-0.117, 1.0],
        [-10.0, 10.0],
        [-10.0, 10.0],
        [-10.0, 10.0],
    ]

    velocity_from_pf = True

    prediction_window_s = 1.0

    pf = ParticleFilter(FilterSerial("power_cell", "0"), num_particles, meas_std_val, u_std, stale_filter_time, bounds)

    x_width = 10.0
    y_width = 10.0
    z_width = 3.0
    # plotter = ParticleFilterPlotter3D(x_width, y_width, z_width)
    # plotter = ParticleFilterPlotter2D(x_width, y_width, tf_to_odom=True)
    plotter = ParticleFilterPredictionPlotter2D(x_width, y_width)

    input_u = InputVector(stale_filter_time)

    pf_delta_velocity = DeltaMeasurement()
    predictor = BouncePredictor(
        rho=0.75,
        tau=0.05,
        g=-9.81,
        a_friction=-0.1,
        t_step=0.001,
        ground_plane=-0.1
    )

    sim_start_t = states[0].stamp
    real_start_t = time.time()

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
        if state.type == "odom":
            if not pf.is_initialized():
                continue
            dt = input_u.odom_update(state)
            vector = input_u.get_vector()
            pf.predict(vector, dt)
            plotter.update_odom(state)
        elif state.type in OBJECT_NAMES:
            state = input_u.meas_update(state)
            meas_z = np.array([state.x, state.y, state.z, state.vx, state.vy, state.vz])
            # print(("%0.3f\t" * len(meas_z)) % tuple(meas_z))

            if not pf.is_initialized():
                print("initializing with %s" % meas_z)
                pf.create_uniform_particles(meas_z, initial_range)
            else:
                pf.update(meas_z)
            plotter.update_measure(state.type, state)
        pf.check_resample()

        pf_state = pf.get_state()
        pf_state.stamp = sim_time
        if input_u.odom_state.stamp > 0.0 and type(plotter) == ParticleFilterPredictionPlotter2D:
            future_state = predictor.get_prediction(pf_state.relative_to(input_u.odom_state), prediction_window_s)
            plotter.update_future_state(future_state.stamp - sim_start_t, future_state)
        if velocity_from_pf:
            plot_state = None
        else:
            plot_state = pf_delta_velocity.update(pf_state)

        if sim_duration >= real_duration:  # if simulation time has caught up to real time, spend some time drawing
            plotter.draw(sim_duration, pf, "pf", plot_state)
            plotter.pause()

    plotter.stop()


if __name__ == "__main__":
    main()
