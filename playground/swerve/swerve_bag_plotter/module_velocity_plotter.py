import csv
import numpy as np
import matplotlib.pyplot as plt


class ModuleVelocityPlotter:
    def __init__(self, max_wheel_velocity, max_azimuth_velocity, max_azimuth_angle, plot_delay=0.01, history=None, num_modules=4, playback_real_time=True):
        self.num_modules = num_modules
        self.plot_delay = plot_delay
        self.fig1 = None
        self.fig2 = None
        self.axes1 = []
        self.axes2 = []
        self.axes3 = []
        self.axes4 = []
        self.history = history
        self.is_paused = False
        self.playback_real_time = playback_real_time

        self.wheel_radius = 0.04445

        self.max_wheel_velocity = max_wheel_velocity
        self.max_azimuth_velocity = max_azimuth_velocity
        self.max_azimuth_angle = max_azimuth_angle

        self.states = []
        self.state_timestamps = []
        self.velocities_lines = []
        self.azimuth_wrap_history = np.array([])
        self.azimuth_ref_wrap_history = np.array([])
        self.wrap_history_len = 10

        self.references = []
        self.state_lines = []
        self.references_lines = []

        self.wheel_velocity_over_time_lines = []
        self.wheel_velocity_ref_over_time_lines = []
        self.azimuth_over_time_lines = []
        self.azimuth_ref_over_time_lines = []
        self.azimuth_delta_over_time_lines = []

        self.volt_timestamps = []
        self.hi_voltages = []
        self.hi_voltages_ref = []
        self.lo_voltages = []
        self.lo_voltages_ref = []
        self.hi_voltages_lines = []
        self.hi_voltages_ref_lines = []
        self.lo_voltages_lines = []
        self.lo_voltages_ref_lines = []

        
        self.physical_boundary = []
        for _ in range(self.num_modules):
            self.physical_boundary.append(np.array([
                [-self.max_wheel_velocity, 0.0],
                [0.0, -self.max_azimuth_velocity],
                [self.max_wheel_velocity, 0.0],
                [0.0, self.max_azimuth_velocity],
                [-self.max_wheel_velocity, 0.0],
            ]))
    
    def draw_perimeter_from_file(self, path):
        self.physical_boundary = [[] for _ in range(self.num_modules)]
        with open(path) as file:
            reader = csv.reader(file)
            header = next(reader)
            for row in reader:
                for index in range(0, len(row), 2):
                    wheel_velocity = float(row[index])
                    azimuth_velocity = float(row[index + 1])
                    module_index = index // 2
                    self.physical_boundary[module_index].append([wheel_velocity, azimuth_velocity])
        self.physical_boundary = [np.array(module_boundary) for module_boundary in self.physical_boundary]

    def on_press(self, event):
        if event.key == ' ':
            self.is_paused = not self.is_paused

    def init(self):
        self.fig1 = plt.figure(1)
        self.fig1.canvas.mpl_connect('key_press_event', self.on_press)

        for index in range(self.num_modules):
            ax1 = self.fig1.add_subplot(2, 2, index + 1)
            self.axes1.append(ax1)

            self.draw_command_boundaries(index)
            self.draw_physical_boundaries(index)

            self.velocities_lines.append(
                ax1.plot([], [], '.', label="velocities")[0]
            )
            ax1.set_title("Module %s" % index)
            ax1.set_xlabel("wheel velocity (m/s)")
            ax1.set_ylabel("azimuth velocity (rad/s)")
            self.states.append(([], [], []))
        plt.tight_layout()
        
        self.fig2 = plt.figure(2)
        self.fig2.canvas.mpl_connect('key_press_event', self.on_press)
        for index in range(self.num_modules):
            ax2_hi = self.fig2.add_subplot(4, 2, 2 * index + 1)
            ax2_lo = self.fig2.add_subplot(4, 2, 2 * index + 2)
            self.axes2.append(ax2_hi)
            self.axes2.append(ax2_lo)

            self.hi_voltages_lines.append(ax2_hi.plot([], [], '-', label="hi")[0])
            self.hi_voltages_ref_lines.append(ax2_hi.plot([], [], '-', label="hi_ref")[0])
            self.lo_voltages_lines.append(ax2_lo.plot([], [], '-', label="lo")[0])
            self.lo_voltages_ref_lines.append(ax2_lo.plot([], [], '-', label="lo_ref")[0])
            self.volt_timestamps.append([])
            self.hi_voltages.append([])
            self.hi_voltages_ref.append([])
            self.lo_voltages.append([])
            self.lo_voltages_ref.append([])

            ax2_hi.set_title("Module %s hi" % index)
            ax2_hi.set_xlabel("time (s)")
            ax2_hi.set_ylabel("voltage (V)")
            ax2_lo.set_title("Module %s lo" % index)
            ax2_lo.set_xlabel("time (s)")
            ax2_lo.set_ylabel("voltage (V)")

        self.fig3 = plt.figure(3)
        self.fig3.canvas.mpl_connect('key_press_event', self.on_press)
        for index in range(self.num_modules):
            ax3 = self.fig3.add_subplot(2, 2, index + 1)
            self.axes3.append(ax3)

            self.state_lines.append(
                ax3.plot([], [], '.', label="state")[0]
            )
            self.references_lines.append(
                ax3.plot([], [], '.', label="reference")[0]
            )
            self.references.append(([], []))

            ax3.set_title("Module %s state" % index)
            ax3.set_xlabel("wheel velocity (m/s)")
            ax3.set_ylabel("azimuth angle (rad)")


        self.fig4 = plt.figure(4)
        self.fig4.canvas.mpl_connect('key_press_event', self.on_press)
        for index in range(self.num_modules):
            ax4_wh = self.fig4.add_subplot(4, 3, 3 * index + 1)
            ax4_az = self.fig4.add_subplot(4, 3, 3 * index + 2)
            ax4_del = self.fig4.add_subplot(4, 3, 3 * index + 3)
            self.axes4.append(ax4_wh)
            self.axes4.append(ax4_az)
            self.axes4.append(ax4_del)

            self.wheel_velocity_over_time_lines.append(ax4_wh.plot([], [], '-', label="wheel vel")[0])
            self.wheel_velocity_ref_over_time_lines.append(ax4_wh.plot([], [], '-', label="wheel ref")[0])
            self.azimuth_over_time_lines.append(ax4_az.plot([], [], '-', label="azimuth")[0])
            self.azimuth_ref_over_time_lines.append(ax4_az.plot([], [], '-', label="azimuth ref")[0])
            self.azimuth_delta_over_time_lines.append(ax4_del.plot([], [], '-', label="azimuth delta")[0])
            self.state_timestamps.append([])

            ax4_wh.set_title("Module %s wheel velocity" % index)
            ax4_wh.set_xlabel("time (s)")
            ax4_wh.set_ylabel("wheel velocity (m/s)")
            ax4_az.set_title("Module %s azimuth" % index)
            ax4_az.set_xlabel("time (s)")
            ax4_az.set_ylabel("azimuth (rad)")
            ax4_del.set_title("Module %s azimuth delta" % index)
            ax4_del.set_xlabel("time (s)")
            ax4_del.set_ylabel("azimuth (rad)")
        if self.playback_real_time:
            plt.ion()
            # plt.legend(loc="best")
            self.fig1.show()
            self.fig2.show()
            self.fig3.show()
            self.fig4.show()
    
    def draw_command_boundaries(self, index):
        ax = self.axes1[index]

        color = "red"
        points = [
            [-self.max_wheel_velocity, -self.max_azimuth_velocity],
            [self.max_wheel_velocity, -self.max_azimuth_velocity],
            [self.max_wheel_velocity, self.max_azimuth_velocity],
            [-self.max_wheel_velocity, self.max_azimuth_velocity],
            [-self.max_wheel_velocity, -self.max_azimuth_velocity],
        ]
        points = np.array(points)
        ax.plot(points[:, 0], points[:, 1], linestyle='-', marker='', color=color, label="command boundary")

    def draw_physical_boundaries(self, index):
        ax = self.axes1[index]
        color = "blue"
        ax.plot(self.physical_boundary[index][:, 0], self.physical_boundary[index][:, 1], linestyle='-', marker='', color=color, label="physical boundary")
    
    def draw(self, timestamp, index, module_data):
        wheel_velocity = module_data["wheel_velocity"]
        azimuth_velocity = module_data["azimuth_velocity"]
        azimuth_angle = module_data["azimuth"]
        wheel_velocity_ref = module_data["wheel_velocity_ref"]
        azimuth_ref = module_data["azimuth_ref"]
        hi_voltage_ref = module_data["hi_voltage_ref"]
        lo_voltage_ref = module_data["lo_voltage_ref"]
        hi_voltage = module_data["hi_voltage"]
        lo_voltage = module_data["lo_voltage"]
        
        wheel_velocities = self.states[index][0]
        azimuth_velocities = self.states[index][1]
        azimuth_angles = self.states[index][2]


        # unwrapped_azimuth_angle = None
        # if azimuth_angle is not None:
        #     self.azimuth_wrap_history = np.append(self.azimuth_wrap_history, azimuth_angle)
        #     if len(self.azimuth_wrap_history) >= self.wrap_history_len:
        #         self.azimuth_wrap_history = np.delete(self.azimuth_wrap_history, 0)

        # unwrapped_azimuth_ref = None
        # if azimuth_ref is not None:
        #     self.azimuth_ref_wrap_history = np.append(self.azimuth_ref_wrap_history, azimuth_angle)
        #     if len(self.azimuth_ref_wrap_history) >= self.wrap_history_len:
        #         self.azimuth_ref_wrap_history = np.delete(self.azimuth_ref_wrap_history, 0)

        if wheel_velocity is not None and azimuth_velocity is not None and azimuth_angle is not None:
            wheel_velocities.append(wheel_velocity)
            azimuth_velocities.append(azimuth_velocity)
            azimuth_angles.append(azimuth_angle)

        state_timestamps = self.state_timestamps[index]
        wheel_references = self.references[index][0]
        azimuth_references = self.references[index][1]

        if wheel_velocity_ref is not None and azimuth_ref is not None:
            state_timestamps.append(timestamp)
            wheel_references.append(wheel_velocity_ref * self.wheel_radius)
            azimuth_references.append(azimuth_ref)

        self.volt_timestamps[index].append(timestamp)
        self.hi_voltages[index].append(hi_voltage)
        self.hi_voltages_ref[index].append(hi_voltage_ref)
        self.lo_voltages[index].append(lo_voltage)
        self.lo_voltages_ref[index].append(lo_voltage_ref)

        if self.history is not None:
            if len(wheel_velocities) > self.history:
                wheel_velocities.pop(0)
            if len(azimuth_velocities) > self.history:
                azimuth_velocities.pop(0)
            if len(azimuth_angles) > self.history:
                azimuth_angles.pop(0)
            if len(wheel_references) > self.history:
                wheel_references.pop(0)
            if len(azimuth_references) > self.history:
                azimuth_references.pop(0)
            if len(state_timestamps) > self.history:
                state_timestamps.pop(0)
        
        if self.playback_real_time:
            self.set_axes(index)
        
    def set_axes(self, index):
        wheel_velocities = self.states[index][0]
        azimuth_velocities = self.states[index][1]
        azimuth_angles = self.states[index][2]

        state_timestamps = self.state_timestamps[index]
        wheel_references = self.references[index][0]
        azimuth_references = self.references[index][1]

        self.velocities_lines[index].set_xdata(wheel_velocities)
        self.velocities_lines[index].set_ydata(azimuth_velocities)

        self.state_lines[index].set_xdata(wheel_velocities)
        self.state_lines[index].set_ydata(azimuth_angles)
        self.references_lines[index].set_xdata(wheel_references)
        self.references_lines[index].set_ydata(azimuth_references)

        self.hi_voltages_lines[index].set_xdata(self.volt_timestamps[index])
        self.hi_voltages_lines[index].set_ydata(self.hi_voltages[index])

        self.hi_voltages_ref_lines[index].set_xdata(self.volt_timestamps[index])
        self.hi_voltages_ref_lines[index].set_ydata(self.hi_voltages_ref[index])

        self.lo_voltages_lines[index].set_xdata(self.volt_timestamps[index])
        self.lo_voltages_lines[index].set_ydata(self.lo_voltages[index])

        self.lo_voltages_ref_lines[index].set_xdata(self.volt_timestamps[index])
        self.lo_voltages_ref_lines[index].set_ydata(self.lo_voltages_ref[index])

        self.wheel_velocity_over_time_lines[index].set_xdata(state_timestamps)
        self.wheel_velocity_over_time_lines[index].set_ydata(wheel_velocities)
        self.wheel_velocity_ref_over_time_lines[index].set_xdata(state_timestamps)
        self.wheel_velocity_ref_over_time_lines[index].set_ydata(wheel_references)

        azimuth_angles = np.array(azimuth_angles)
        azimuth_references = np.array(azimuth_references)
        state_timestamps = np.array(state_timestamps)

        # indices = np.where((15.0 < state_timestamps) & (state_timestamps < 24.0))[0]
        # indices = np.where((8.0 < state_timestamps) & (state_timestamps < 12.0))[0]
        # azimuth_angles = azimuth_angles[indices]
        # azimuth_references = azimuth_references[indices]
        # state_timestamps = state_timestamps[indices]

        # azimuth_angles = np.unwrap(azimuth_angles + np.pi)
        # azimuth_references = np.unwrap(azimuth_references + np.pi)

        azimuth_delta_angles = azimuth_references - azimuth_angles
        # azimuth_delta_angles = np.unwrap(azimuth_delta_angles)
        # azimuth_delta_angles = np.mod(azimuth_delta_angles + 2 * np.pi, 2 * np.pi)

        # self.azimuth_over_time_lines[index].set_xdata(state_timestamps)
        # self.azimuth_over_time_lines[index].set_ydata(azimuth_delta_angles)
        self.azimuth_over_time_lines[index].set_xdata(state_timestamps)
        self.azimuth_over_time_lines[index].set_ydata(azimuth_angles)
        self.azimuth_ref_over_time_lines[index].set_xdata(state_timestamps)
        self.azimuth_ref_over_time_lines[index].set_ydata(azimuth_references)
        self.azimuth_delta_over_time_lines[index].set_xdata(state_timestamps)
        self.azimuth_delta_over_time_lines[index].set_ydata(azimuth_delta_angles)

        ax1 = self.axes1[index]
        ax1.legend(loc="best", bbox_to_anchor=(1, 0))
        ax1.set_xlim(self.get_limits(wheel_velocities, -self.max_wheel_velocity, self.max_wheel_velocity))
        ax1.set_ylim(self.get_limits(azimuth_velocities, -self.max_azimuth_velocity, self.max_azimuth_velocity))

        ax2_hi = self.axes2[2 * index]
        ax2_lo = self.axes2[2 * index + 1]
        ax2_hi.legend(loc="best", bbox_to_anchor=(0, 0))
        ax2_lo.legend(loc="best", bbox_to_anchor=(0, 0))
        x_limits = self.get_limits(self.volt_timestamps[index])
        ax2_hi.set_xlim(x_limits)
        ax2_hi.set_ylim(self.get_limits(self.hi_voltages[index], self.hi_voltages_ref[index], -12, 12))
        ax2_lo.set_xlim(x_limits)
        ax2_lo.set_ylim(self.get_limits(self.lo_voltages[index], self.lo_voltages_ref[index], -12, 12))

        ax3 = self.axes3[index]
        ax3.legend(loc="best", bbox_to_anchor=(1, 0))
        ax3.set_xlim(self.get_limits(wheel_references, wheel_references, -self.max_wheel_velocity, self.max_wheel_velocity))
        ax3.set_ylim(self.get_limits(azimuth_angles, azimuth_references, -self.max_azimuth_angle, self.max_azimuth_angle))

        ax4_wh = self.axes4[3 * index]
        ax4_az = self.axes4[3 * index + 1]
        ax4_del = self.axes4[3 * index + 2]
        ax4_wh.legend(loc="best", bbox_to_anchor=(0, 0))
        ax4_az.legend(loc="best", bbox_to_anchor=(0, 0))
        ax4_del.legend(loc="best", bbox_to_anchor=(0, 0))
        x_limits = self.get_limits(state_timestamps)
        ax4_wh.set_xlim(x_limits)
        ax4_wh.set_ylim(self.get_limits(wheel_references, wheel_references, -self.max_wheel_velocity, self.max_wheel_velocity))
        ax4_az.set_xlim(x_limits)
        ax4_az.set_ylim(self.get_limits(azimuth_angles, azimuth_references, -self.max_azimuth_angle, self.max_azimuth_angle))
        ax4_del.set_xlim(x_limits)
        ax4_del.set_ylim(self.get_limits(azimuth_delta_angles, -self.max_azimuth_angle, self.max_azimuth_angle))


    def get_limits(self, *args):
        minimum = None
        maximum = None
        for arg in args:
            if type(arg) == list or type(arg) == tuple or type(arg) == np.ndarray:
                if len(arg) == 0:
                    continue
                try:
                    arg_min = min(arg)
                    arg_max = max(arg)
                except TypeError:
                    continue
            else:
                arg_min = arg
                arg_max = arg
            if arg_min is None or arg_max is None:
                continue
            if minimum is None or arg_min < minimum:
                minimum = arg_min
            if maximum is None or arg_max > maximum:
                maximum = arg_max
        return minimum, maximum

    
    def clear(self):
        # plt.cla()
        for ax in self.axes1:
            ax.clear()

    def pause(self):
        self.fig1.canvas.start_event_loop(self.plot_delay)
        self.fig1.canvas.flush_events()
        # self.fig.canvas.draw()
        # plt.pause(self.plot_delay)

    def stop(self):
        for index in range(self.num_modules):
            self.set_axes(index)
        plt.ioff()
        plt.show()
