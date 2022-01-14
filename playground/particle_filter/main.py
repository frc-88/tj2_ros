import time
import numpy as np
from jit_particle_filter import JitParticleFilter
from particle_filter import ParticleFilter, FilterSerial
from state import State, OBJECT_NAMES
import state_loader
from plotter import ParticleFilterPlotter3D, ParticleFilterPlotter2D


class DeltaTimer:
    def __init__(self):
        self.prev_stamp = None

    def dt(self, timestamp):
        if self.prev_stamp is None:
            self.prev_stamp = timestamp
        dt = timestamp - self.prev_stamp
        self.prev_stamp = timestamp
        return dt


class VelocityFilter:
    def __init__(self, k):
        self.k = k
        self.prev_value = None
        self.speed = 0.0

    def update(self, dt, value):
        if self.prev_value is None:
            self.prev_value = value
        raw_delta = (value - self.prev_value) / dt
        self.prev_value = value
        if self.k is None:
            self.speed = raw_delta
        else:
            self.speed = self.k * (raw_delta - self.speed)
        return self.speed


class DeltaMeasurement:
    def __init__(self):
        smooth_k = None
        self.vx_filter = VelocityFilter(smooth_k)
        self.vy_filter = VelocityFilter(smooth_k)
        self.vz_filter = VelocityFilter(smooth_k)
        self.timer = DeltaTimer()

    def update(self, state: State):
        dt = self.timer.dt(state.stamp)
        if dt == 0.0:
            return state
        new_state = State.from_state(state)
        new_state.vx = self.vx_filter.update(dt, state.x)
        new_state.vy = self.vy_filter.update(dt, state.y)
        new_state.vz = self.vz_filter.update(dt, state.z)
        return new_state


class InputVector:
    def __init__(self):
        self.odom_timer = DeltaTimer()
        # self.meas = {}
        self.meas = DeltaMeasurement()
        self.meas_input = np.array([0.0, 0.0, 0.0, 0.0])
        self.vector = np.array([0.0, 0.0, 0.0, 0.0])

    def odom_update(self, odom_state: State):
        dt = self.odom_timer.dt(odom_state.stamp)
        self.vector = np.array([odom_state.vx, odom_state.vy, odom_state.vz, odom_state.vt])
        self.vector += self.meas_input
        return dt

    def meas_update(self, meas_state: State):
        # TODO: add trackers for each instance of an object
        # name = meas_state.type
        # if name not in self.meas:
        #     self.meas[name] = DeltaMeasurement()
        # self.meas[name].update(meas_state)
        new_state = self.meas.update(meas_state)
        self.meas_input = np.array([new_state.vx, new_state.vy, new_state.vz, new_state.vt])

    def get_vector(self):
        return self.vector


def main():
    repickle = False
    # path = "./data/detections_2022-01-14-12-02-32.json"
    # path = "./data/detections_2022-01-14-12-39-34.json"
    # path = "./data/detections_2022-01-14-13-42-43.json"
    path = "./data/detections_2022-01-14-14-05-17.json"

    states = state_loader.read_pkl(path, repickle)

    meas_std_val = 0.05
    u_std = [0.007, 0.007, 0.007, 0.007]
    initial_range = [1.0, 1.0, 1.0]
    ground_plane = 0.1

    is_grounded = True

    pf = ParticleFilter(FilterSerial("power_cell", "0"), 50, meas_std_val, u_std, 1.0, -0.1)

    x_width = 10.0
    y_width = 10.0
    z_width = 3.0
    plotter = ParticleFilterPlotter3D(x_width, y_width, z_width)
    # plotter = ParticleFilterPlotter2D(x_width, y_width)

    sim_start_t = states[0].stamp
    real_start_t = time.time()

    input_u = InputVector()

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
            pf.predict(vector, dt, is_grounded)
            plotter.update_odom(state)
        elif state.type in OBJECT_NAMES:
            input_u.meas_update(state)
            meas_z = [state.x, state.y, state.z]
            print(("%0.3f\t" * len(meas_z)) % tuple(meas_z))

            if not pf.is_initialized():
                print("initializing with %s" % meas_z)
                pf.create_uniform_particles(meas_z, initial_range)
            else:
                pf.update(meas_z)
            plotter.update_measure(state.type, state)
        pf.check_resample()

        estimated_state = pf.mean()
        is_grounded = estimated_state[2] < ground_plane
        # print(estimated_state[3:6])

        if sim_duration >= real_duration:  # if simulation time has caught up to real time, spend some time drawing
            plotter.clear()
            plotter.draw(pf, "pf")
            plotter.pause()

    plotter.stop()


if __name__ == "__main__":
    main()
