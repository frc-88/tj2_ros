import time
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


class DeltaMeasurement:
    def __init__(self):
        self.prev_state = None
        self.timer = DeltaTimer()

    def update(self, stamp, state):
        if self.prev_state is None:
            self.prev_state = state
            return state
        dt = self.timer.dt(stamp)
        if dt == 0.0:
            return state
        new_state = State.from_state(state)
        new_state.vx = (state.x - self.prev_state.x) / dt
        new_state.vy = (state.y - self.prev_state.y) / dt
        new_state.vz = (state.z - self.prev_state.z) / dt
        self.prev_state = state
        return new_state


def main():
    repickle = False
    # path = "./data/detections_2022-01-14-12-02-32.json"
    path = "./data/detections_2022-01-14-12-39-34.json"

    states = state_loader.read_pkl(path, repickle)

    meas_std_val = 0.004
    u_std = [0.004, 0.004, 0.004, 0.004]
    initial_range = [1.0, 1.0, 1.0, 0.25, 0.25, 0.25]
    ground_plane = 0.1

    is_grounded = True

    pf = ParticleFilter(FilterSerial("power_cell", "0"), 50, meas_std_val, u_std, 1.0, -0.1)

    x_width = 10.0
    y_width = 10.0
    z_width = 3.0
    plotter = ParticleFilterPlotter3D(x_width, y_width, z_width)
    # plotter = ParticleFilterPlotter2D(x_width, y_width)

    d_meas = {}
    timer = DeltaTimer()

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
            input_u = [state.vx, state.vy, state.vz, state.vt]
            if not pf.is_initialized():
                continue
            pf.predict(input_u, timer.dt(state.stamp), is_grounded)
            plotter.update_odom(state)
        elif state.type in OBJECT_NAMES:
            if state.type not in d_meas:
                d_meas[state.type] = DeltaMeasurement()
            state = d_meas[state.type].update(state.stamp, state)
            meas_z = [state.x, state.y, state.z, state.vx, state.vy, state.vz]
            print(("%0.3f\t" * 3) % tuple(meas_z[3:6]))

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
