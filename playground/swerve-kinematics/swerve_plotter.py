import math
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Arrow


class SwervePlotter:
    def __init__(self, x_window, y_window, plot_delay=0.01):
        self.plot_delay = plot_delay
        self.fig = None
        self.ax = None

        self.x_window = x_window
        self.y_window = y_window

        self.measured_line = None
        self.calculated_line = None
        self.meas_heading_arrow = None
        self.calc_heading_arrow = None

        self.measured_states = [[], []]
        self.calculated_states = [[], []]

        self.init()

    def init(self):
        self.fig = plt.figure()

        self.ax = self.fig.add_subplot(111)

        self.measured_line = self.ax.plot([0.0], [0.0], marker='.', color='b', label="meas")[0]
        self.calculated_line = self.ax.plot([0.0], [0.0], marker='.', color='r', label="calc")[0]
        # self.heading_arrow = self.ax.arrow(0.0, 0.0, 0.25, 0.25, head_width=0.25, head_length=0.25, fc='gray', ec='gray')
        # self.meas_heading_arrow = self.get_heading_arrow(0.0, 0.0, 0.0, 0.0, 'b')
        # self.calc_heading_arrow = self.get_heading_arrow(0.0, 0.0, 0.0, 0.0, 'r')

        self.set_window()

        plt.tight_layout()
        plt.ion()
        plt.legend()
        self.fig.show()
    
    def get_heading_arrow(self, x0, y0, x1, y1, color):
        arrow = Arrow(x0, y0, x1, y1, color=color)  #head_width=0.25, head_length=0.25, fc='gray', ec='gray')
        return self.ax.add_patch(arrow)

    def set_window(self):
        self.ax.set_xlim(-self.x_window / 2, self.x_window / 2)
        self.ax.set_ylim(-self.y_window / 2, self.y_window / 2)

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
    
    def set_measured_arrow(self, theta, arrow_length=2.0):
        x0 = self.measured_states[0][-1]
        y0 = self.measured_states[1][-1]
        x1 = arrow_length * math.cos(theta)
        y1 = arrow_length * math.sin(theta)
        if self.meas_heading_arrow is not None:
            self.meas_heading_arrow.remove()
        self.meas_heading_arrow = self.get_heading_arrow(x0, y0, x1, y1, "b")

    def set_calculated_arrow(self, theta, arrow_length=2.0):
        x0 = self.calculated_states[0][-1]
        y0 = self.calculated_states[1][-1]
        x1 = arrow_length * math.cos(theta)
        y1 = arrow_length * math.sin(theta)
        if self.calc_heading_arrow is not None:
            self.calc_heading_arrow.remove()
        self.calc_heading_arrow = self.get_heading_arrow(x0, y0, x1, y1, "r")
    
    def clear(self):
        # plt.cla()
        self.ax.clear()
        self.set_window()

    def pause(self):
        self.fig.canvas.start_event_loop(self.plot_delay)
        self.fig.canvas.flush_events()
        # self.fig.canvas.draw()
        # plt.pause(self.plot_delay)

    def stop(self):
        plt.ioff()
        plt.show()