import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation

from tj2_tools.robot_state import Simple3DState


class ParticleFilterPlotterBase:
    def __init__(self, tf_to_odom=True, plot_delay=0.01):
        self.plot_delay = plot_delay
        self.tf_to_odom = tf_to_odom
        self.fig = None
        self.ax = None
        self.odom_state = Simple3DState()
        self.meas_states = {}
        self.is_paused = False

        self.init()

    def init(self):
        self.fig = plt.figure()
        self.fig.canvas.mpl_connect('key_press_event', self.on_press)

        plt.tight_layout()
        plt.ion()
        self.fig.show()

    def update_odom(self, state):
        self.odom_state = state

    def update_measure(self, name, state):
        self.meas_states[name] = state

    def draw(self, timestamp, pf, label, state=None):
        raise NotImplemented

    def clear(self):
        plt.cla()

    def on_press(self, event):
        if event.key == ' ':
            self.is_paused = not self.is_paused

    def pause(self):
        self.fig.canvas.start_event_loop(self.plot_delay)
        self.fig.canvas.flush_events()
        # plt.pause(self.plot_delay)

    def base_link_to_odom(self, state, x, y, z):
        point = np.array([x, y, z])
        if self.tf_to_odom:
            angle = state.theta
            # tf_mat = np.array([
            #     [np.cos(angle), -np.sin(angle), state.x],
            #     [np.sin(angle), np.cos(angle), state.y],
            #     [0.0, 0.0,  1.0]
            # ])
            # point = np.array([x, y, 1.0])
            # tf_point = np.dot(tf_mat, point)
            rot_mat = Rotation.from_euler("z", angle)

            tf_point = np.dot(rot_mat.as_matrix(), point)
            tf_point[0] += state.x
            tf_point[1] += state.y
            tf_point[2] += state.z
            return tf_point
        else:
            return point

    def velocity_base_link_to_odom(self, state, vx, vy, vz):
        velocity = np.array([vx, vy, vz])
        if self.tf_to_odom:
            angle = state.theta
            rot_mat = Rotation.from_euler("z", angle)

            tf_vel = np.dot(rot_mat.as_matrix(), velocity)
            tf_vel[0] += state.vx
            tf_vel[1] += state.vy
            tf_vel[2] += state.vz
            return tf_vel
        else:
            return velocity

    def stop(self):
        plt.ioff()
        plt.show()


class ParticleFilterPlotter3D(ParticleFilterPlotterBase):
    def __init__(self, x_window, y_window, z_window, tf_to_odom=True, plot_delay=0.01):
        self.x_window = x_window
        self.y_window = y_window
        self.z_window = z_window
        super(ParticleFilterPlotter3D, self).__init__(tf_to_odom, plot_delay)

    def init(self):
        super(ParticleFilterPlotter3D, self).init()
        self.ax = self.fig.add_subplot(111, projection='3d')

    def draw(self, timestamp, pf, label, state=None):
        # try:
        #     mu, var = pf.estimate()
        # except ZeroDivisionError:
        #     return
        if state is None:
            state = pf.get_state()
        state_at_odom = state.relative_to(self.odom_state)

        particles = self.base_link_to_odom(self.odom_state, pf.particles[:, 0], pf.particles[:, 1],
                                           pf.particles[:, 2])
        self.ax.clear()
        self.ax.scatter(particles[0], particles[1], particles[2], marker='.', s=1, color='k', alpha=0.5)
        self.ax.set_xlim(-self.x_window / 2, self.x_window / 2)
        self.ax.set_ylim(-self.y_window / 2, self.y_window / 2)
        self.ax.set_zlim(0.0, self.z_window)

        self.ax.scatter(state_at_odom.x, state_at_odom.y, state_at_odom.z, color='g', s=25)

        self.ax.scatter(self.odom_state.x, self.odom_state.y, self.odom_state.z, marker='*', color='b', s=25)

        for name, meas_state in self.meas_states.items():
            if self.odom_state.stamp - meas_state.stamp < 0.25:
                odom_meas = meas_state.relative_to(self.odom_state)
                self.ax.scatter(odom_meas.x, odom_meas.y, odom_meas.z, marker='*', color='r', s=25)

        # self.ax.text(odom_mu[0], odom_mu[1], odom_mu[2], label, transform=self.ax.transAxes, fontsize=14,
        #              verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        self.ax.text(state_at_odom.x, state_at_odom.y, state_at_odom.z, label, color="black")


class ParticleFilterPlotter2D(ParticleFilterPlotterBase):
    def __init__(self, x_window, y_window, tf_to_odom=True, plot_delay=0.01):
        self.x_window = x_window
        self.y_window = y_window
        self.meas_vx_line = None
        self.meas_vy_line = None
        self.state_vx_line = None
        self.state_vy_line = None

        self.ax_x_vel = None
        self.ax_y_vel = None
        self.ax_z_vel = None

        self.timestamps = []
        self.x_vel_data = []
        self.y_vel_data = []
        self.z_vel_data = []

        self.meas_timestamps = []
        self.meas_x_vel_data = []
        self.meas_y_vel_data = []
        self.meas_z_vel_data = []

        super(ParticleFilterPlotter2D, self).__init__(tf_to_odom, plot_delay)

    def init(self):
        super(ParticleFilterPlotter2D, self).init()
        self.ax = self.fig.add_subplot(3, 1, 1)
        self.ax_x_vel = self.fig.add_subplot(3, 1, 2)
        self.ax_y_vel = self.fig.add_subplot(3, 1, 3)
        self.meas_vx_line = self.ax_x_vel.plot([], [], label="meas vx")[0]
        self.state_vx_line = self.ax_x_vel.plot([], [], label="vx")[0]

        self.meas_vy_line = self.ax_y_vel.plot([], [], label="meas vy")[0]
        self.state_vy_line = self.ax_y_vel.plot([], [], label="vy")[0]
        self.ax_x_vel.legend(loc=2)
        self.ax_y_vel.legend(loc=2)

    def draw(self, timestamp, pf, label, state=None):
        # try:
        #     mu, var = pf.estimate()
        # except ZeroDivisionError:
        #     return
        #
        # x, y, z, vx, vy, vz = mu
        # state = Simple3DState(x, y, z, 0.0, vx, vy, vz, 0.0)
        if state is None:
            state = pf.get_state()
        state_at_odom = state.relative_to(self.odom_state)

        self.ax.clear()
        particles = self.base_link_to_odom(
            self.odom_state,
            pf.particles[:, 0], pf.particles[:, 1], pf.particles[:, 2]
        )

        if self.tf_to_odom:
            # state_velocities = self.velocity_base_link_to_odom(
            #     self.odom_state,
            #     vx, vy, vz
            # )
            state_velocities = [state_at_odom.vx, state_at_odom.vy, state_at_odom.vz]
        else:
            state_velocities = [state.vx, state.vy, state.vz]

        self.ax.scatter(particles[0], particles[1], marker='.', s=1, color='k', alpha=0.5)
        self.ax.set_xlim(-self.x_window / 2, self.x_window / 2)
        self.ax.set_ylim(-self.y_window / 2, self.y_window / 2)

        # odom_mu = self.base_link_to_odom(self.odom_state, x, y, z)
        self.ax.scatter(state_at_odom.x, state_at_odom.y, color='g', s=25)
        self.ax.text(state_at_odom.x, state_at_odom.y, label, color="black")

        self.ax.scatter(self.odom_state.x, self.odom_state.y, marker='*', color='b', s=25)
        self.ax.scatter(0.0, 0.0, marker='*', color='k', s=25)

        for name, meas_state in self.meas_states.items():
            if self.odom_state.stamp - meas_state.stamp < 1.0:
                odom_meas = meas_state.relative_to(self.odom_state)
                self.ax.scatter(odom_meas.x, odom_meas.y, marker='*', color='r', s=25)
                # odom_meas = self.base_link_to_odom(self.odom_state, meas_state.x, meas_state.y, meas_state.z)
                # self.ax.scatter(odom_meas[0], odom_meas[1], marker='*', color='r', s=25)
                # meas_velocities = self.velocity_base_link_to_odom(
                #     self.odom_state,
                #     meas_state.vx, meas_state.vy, meas_state.vz
                # )
                self.meas_timestamps.append(timestamp)
                self.meas_x_vel_data.append(odom_meas.vx)
                self.meas_y_vel_data.append(odom_meas.vy)

        # all_x = np.append(self.x_vel_data, self.meas_x_vel_data)
        # all_y = np.append(self.y_vel_data, self.meas_y_vel_data)
        all_x = self.x_vel_data
        all_y = self.y_vel_data

        self.timestamps.append(timestamp)
        self.x_vel_data.append(state_velocities[0])
        self.y_vel_data.append(state_velocities[1])
        # self.z_vel_data.append(tfd_velocities[2])
        self.meas_vx_line.set_xdata(self.meas_timestamps)
        self.meas_vx_line.set_ydata(self.meas_x_vel_data)
        self.state_vx_line.set_xdata(self.timestamps)
        self.state_vx_line.set_ydata(self.x_vel_data)
        self.ax_x_vel.set_xlim(np.min(self.timestamps), np.max(self.timestamps) + 0.5)
        self.ax_x_vel.set_ylim(np.min(all_x), np.max(all_x))

        self.meas_vy_line.set_xdata(self.meas_timestamps)
        self.meas_vy_line.set_ydata(self.meas_y_vel_data)
        self.state_vy_line.set_xdata(self.timestamps)
        self.state_vy_line.set_ydata(self.y_vel_data)
        self.ax_y_vel.set_xlim(np.min(self.timestamps), np.max(self.timestamps) + 0.5)
        self.ax_y_vel.set_ylim(np.min(all_y), np.max(all_y))


class ParticleFilterPredictionPlotter2D(ParticleFilterPlotterBase):
    def __init__(self, x_window, y_window, tf_to_odom=True, plot_delay=0.01):
        self.x_window = x_window
        self.y_window = y_window
        self.x_line = None
        self.y_line = None
        self.z_line = None
        self.future_x_line = None
        self.future_y_line = None
        self.future_z_line = None

        self.ax_x = None
        self.ax_y = None
        self.ax_z = None

        self.future_times = []
        self.future_x = []
        self.future_y = []
        self.future_z = []

        self.state_times = []
        self.state_x = []
        self.state_y = []
        self.state_z = []

        self.max_time = 0.0
        self.max_x = 0.0
        self.max_y = 0.0
        self.max_z = 0.0

        self.min_time = 0.0
        self.min_x = 0.0
        self.min_y = 0.0
        self.min_z = 0.0

        super(ParticleFilterPredictionPlotter2D, self).__init__(tf_to_odom, plot_delay)

    def init(self):
        super(ParticleFilterPredictionPlotter2D, self).init()
        self.ax = self.fig.add_subplot(2, 2, 1)
        self.ax_x = self.fig.add_subplot(2, 2, 2)
        self.ax_y = self.fig.add_subplot(2, 2, 3)
        self.ax_z = self.fig.add_subplot(2, 2, 4)
        self.x_line = self.ax_x.plot([], [], label="x")[0]
        self.future_x_line = self.ax_x.plot([], [], label="future x")[0]

        self.y_line = self.ax_y.plot([], [], label="y")[0]
        self.future_y_line = self.ax_y.plot([], [], label="future y")[0]

        self.z_line = self.ax_z.plot([], [], label="z")[0]
        self.future_z_line = self.ax_z.plot([], [], label="future z")[0]

        self.ax_x.legend(loc=2)
        self.ax_y.legend(loc=2)
        self.ax_z.legend(loc=2)

    def update_future_state(self, timestamp, state: Simple3DState):
        self.future_times.append(timestamp)
        self.future_x.append(state.x)
        self.future_y.append(state.y)
        self.future_z.append(state.z)

        self.max_time = max(self.max_time, np.max(self.future_times))
        self.max_x = max(self.max_x, max(self.future_x))
        self.max_y = max(self.max_y, max(self.future_y))
        self.max_z = max(self.max_z, max(self.future_z))

        self.min_time = min(self.min_time, np.min(self.future_times))
        self.min_x = min(self.min_x, min(self.future_x))
        self.min_y = min(self.min_y, min(self.future_y))
        self.min_z = min(self.min_z, min(self.future_z))

    def draw(self, timestamp, pf, label, state=None):
        # try:
        #     mu, var = pf.estimate()
        # except ZeroDivisionError:
        #     return
        #
        # x, y, z, vx, vy, vz = mu
        # state = Simple3DState(x, y, z, 0.0, vx, vy, vz, 0.0)
        if state is None:
            state = pf.get_state()
        state_at_odom = state.relative_to(self.odom_state)

        self.ax.clear()
        particles = self.base_link_to_odom(
            self.odom_state,
            pf.particles[:, 0], pf.particles[:, 1], pf.particles[:, 2]
        )

        self.ax.scatter(particles[0], particles[1], marker='.', s=1, color='k', alpha=0.5)
        self.ax.set_xlim(-self.x_window / 2, self.x_window / 2)
        self.ax.set_ylim(-self.y_window / 2, self.y_window / 2)

        # odom_mu = self.base_link_to_odom(self.odom_state, x, y, z)
        self.ax.scatter(state_at_odom.x, state_at_odom.y, color='g', s=25)
        self.ax.text(state_at_odom.x, state_at_odom.y, label, color="black")

        self.ax.scatter(self.odom_state.x, self.odom_state.y, marker='*', color='b', s=25)
        self.ax.scatter(0.0, 0.0, marker='*', color='k', s=25)

        for name, meas_state in self.meas_states.items():
            if self.odom_state.stamp - meas_state.stamp < 1.0:
                odom_meas = meas_state.relative_to(self.odom_state)
                self.ax.scatter(odom_meas.x, odom_meas.y, marker='*', color='r', s=25)

        self.state_times.append(timestamp)
        self.state_x.append(state_at_odom.x)
        self.state_y.append(state_at_odom.y)
        self.state_z.append(state_at_odom.z)

        self.x_line.set_xdata(self.state_times)
        self.y_line.set_xdata(self.state_times)
        self.z_line.set_xdata(self.state_times)
        self.future_x_line.set_xdata(self.future_times)
        self.future_y_line.set_xdata(self.future_times)
        self.future_z_line.set_xdata(self.future_times)

        self.x_line.set_ydata(self.state_x)
        self.y_line.set_ydata(self.state_y)
        self.z_line.set_ydata(self.state_z)
        self.future_x_line.set_ydata(self.future_x)
        self.future_y_line.set_ydata(self.future_y)
        self.future_z_line.set_ydata(self.future_z)

        self.max_time = max(self.max_time, max(self.state_times))
        self.max_x = max(self.max_x, max(self.state_x))
        self.max_y = max(self.max_y, max(self.state_y))
        self.max_z = max(self.max_z, max(self.state_z))

        self.min_time = min(self.min_time, min(self.state_times))
        self.min_x = min(self.min_x, min(self.state_x))
        self.min_y = min(self.min_y, min(self.state_y))
        self.min_z = min(self.min_z, min(self.state_z))

        self.ax_x.set_xlim(self.min_time, self.max_time)
        self.ax_y.set_xlim(self.min_time, self.max_time)
        self.ax_z.set_xlim(self.min_time, self.max_time)

        self.ax_x.set_ylim(self.min_x, self.max_x)
        self.ax_y.set_ylim(self.min_y, self.max_y)
        self.ax_z.set_ylim(self.min_z, self.max_z)
