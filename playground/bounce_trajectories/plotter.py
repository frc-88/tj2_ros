import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation

from tj2_tools.particle_filter.state import FilterState


class LivePlotterBase:
    def __init__(self, plot_delay=0.01):
        self.plot_delay = plot_delay
        self.fig = None
        self.ax = None
        self.meas_states = {}
        self.is_paused = False

        self.init()

    def init(self):
        self.fig = plt.figure()
        self.fig.canvas.mpl_connect('key_press_event', self.on_press)

        plt.tight_layout()
        plt.ion()
        self.fig.show()

    def update_measure(self, name, state):
        self.meas_states[name] = state

    def draw(self, timestamp, **kwargs):
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

    def stop(self):
        plt.ioff()
        plt.show()


class LivePlotter3D(LivePlotterBase):
    def __init__(self, x_window, y_window, z_window, plot_delay=0.01):
        self.x_window = x_window
        self.y_window = y_window
        self.z_window = z_window
        super(LivePlotter3D, self).__init__(plot_delay)

    def init(self):
        super(LivePlotter3D, self).init()
        self.ax = self.fig.add_subplot(111, projection='3d')

    def draw(self, timestamp, **kwargs):
        # try:
        #     mu, var = pf.estimate()
        # except ZeroDivisionError:
        #     return
        object_state = kwargs["object"]

        self.ax.clear()
        self.ax.scatter(object_state.x, object_state.y, object_state.z, marker='.', s=1, color='k', alpha=0.5)
        self.ax.set_xlim(-self.x_window / 2, self.x_window / 2)
        self.ax.set_ylim(-self.y_window / 2, self.y_window / 2)
        self.ax.set_zlim(0.0, self.z_window)

        self.ax.scatter(0.0, 0.0, 0.0, marker='*', color='b', s=25)


class LivePlotter2D(LivePlotterBase):
    def __init__(self, x_window, y_window, plot_delay=0.01):
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

        super(LivePlotter2D, self).__init__(plot_delay)

    def init(self):
        super(LivePlotter2D, self).init()
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

    def update_future_state(self, state: FilterState):
        if state is None:
            return
        self.future_times.append(state.stamp)
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

    def draw(self, timestamp, **kwargs):
        # try:
        #     mu, var = pf.estimate()
        # except ZeroDivisionError:
        #     return
        #
        # x, y, z, vx, vy, vz = mu
        # state = FilterState(x, y, z, 0.0, vx, vy, vz, 0.0)
        object_state = kwargs["object"]
        odom_state = kwargs["odom"]
        predicted_state = kwargs["prediction"]
        self.update_future_state(predicted_state)

        self.ax.clear()

        self.ax.set_xlim(-self.x_window / 2, self.x_window / 2)
        self.ax.set_ylim(-self.y_window / 2, self.y_window / 2)

        # odom_mu = self.base_link_to_odom(self.odom_state, x, y, z)
        self.ax.scatter(object_state.x, object_state.y, color='b', s=25)
        if len(self.future_x) > 0 and len(self.future_y) > 0:
            self.ax.scatter(self.future_x[-1], self.future_y[-1], color='orange', s=25)
        self.ax.scatter(odom_state.x, odom_state.y, marker='*', color='g', s=25)
        self.ax.scatter(0.0, 0.0, marker='.', color='k', s=25)

        self.state_times.append(timestamp)
        self.state_x.append(object_state.x)
        self.state_y.append(object_state.y)
        self.state_z.append(object_state.z)

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
