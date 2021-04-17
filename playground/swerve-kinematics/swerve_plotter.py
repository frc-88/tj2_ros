import matplotlib.pyplot as plt


class SwervePlotter:
    def __init__(self, x_window, y_window, plot_delay=0.01):
        self.plot_delay = plot_delay
        self.fig = None
        self.ax = None

        self.x_window = x_window
        self.y_window = y_window


        self.measured_line = None
        self.calculated_line = None

        self.measured_states = [[], []]
        self.calculated_states = [[], []]

        self.init()

    def init(self):
        self.fig = plt.figure()

        self.ax = self.fig.add_subplot(111)
        self.measured_line = self.ax.plot([0.0], [0.0], marker='.', color='b', label="meas")[0]
        self.calculated_line = self.ax.plot([0.0], [0.0], marker='.', color='r', label="calc")[0]

        self.ax.set_xlim(-self.x_window / 2, self.x_window / 2)
        self.ax.set_ylim(-self.y_window / 2, self.y_window / 2)

        plt.tight_layout()
        plt.ion()
        plt.legend()
        self.fig.show()

    def append_measured_state(self, x, y):
        self.measured_states[0].append(x)
        self.measured_states[1].append(y)
        self.measured_line.set_xdata(self.measured_states[0])
        self.measured_line.set_ydata(self.measured_states[1])

    def append_calculated_state(self, x, y):
        self.calculated_states[0].append(x)
        self.calculated_states[1].append(y)
        self.calculated_line.set_xdata(self.calculated_states[0])
        self.calculated_line.set_ydata(self.calculated_states[1])

    def clear(self):
        plt.cla()

    def pause(self):
        self.fig.canvas.start_event_loop(self.plot_delay)
        self.fig.canvas.flush_events()
        # plt.pause(self.plot_delay)

    def stop(self):
        plt.ioff()
        plt.show()