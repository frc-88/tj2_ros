import time
import numpy as np
from module_velocity_plotter import ModuleVelocityPlotter
from state_loader import get_states

# data = get_states("data/diffyjr_2022-06-08-23-52-28_0.json")
# data = get_states("data/diffyjr_2022-06-09-22-42-13_0.json")  # perimeter
# data = get_states("data/diffyjr_2022-06-09-23-07-18_0.json")  # reference recorded (wrong units)
# data = get_states("data/diffyjr_2022-06-09-23-41-58_0.json")  # figure 8's

# no angle controller
# data = get_states("data/diffyjr_2022-06-09-23-55-06_0.json")  # 10.5 V
# data = get_states("data/diffyjr_2022-06-11-00-38-10_0.json")  # 10.5 V
# data = get_states("data/diffyjr_2022-06-11-00-44-19_0.json")  # 11.0 V
# data = get_states("data/diffyjr_2022-06-11-00-46-46_0.json")  # 12.0 V

# no voltage recorded
# data = get_states("data/diffyjr_2022-06-11-12-54-00_0.json")
# data = get_states("data/diffyjr_2022-06-11-14-31-53_0.json")
# data = get_states("data/diffyjr_2022-06-11-14-34-45_0.json")
data = get_states("data/diffyjr_2022-06-11-14-39-19_0.json")

playback_real_time = False
# playback_real_time = True
# plot_rate = 1.0 / 10.0
plot_rate = 0.0

plotter = ModuleVelocityPlotter(4.76, 47.37, 2 * np.pi, history=None, playback_real_time=playback_real_time)
plotter.draw_perimeter_from_file("data/perimeter.csv")
plotter.init()

prev_draw_time = time.time()

sim_start_t = data[0][0]["time"]
real_start_t = time.time()

length = min([len(data[module_index]) for module_index in range(len(data))])

for index in range(length):
    current_time = time.time()
    pause_start_t = current_time
    while plotter.is_paused:
        plotter.pause()
        current_time = time.time()
    real_start_t += current_time - pause_start_t
    sim_time = data[0][index]["time"]
    real_time = current_time
    sim_duration = sim_time - sim_start_t
    real_duration = real_time - real_start_t
    for module_index in range(len(data)):
        module_data = data[module_index][index]
        plotter.draw(sim_duration, module_index, module_data)
    
    if playback_real_time:
        if sim_duration >= real_duration:  # if simulation time has caught up to real time, spend some time drawing
            plotter.pause()
    else:
        if plot_rate > 0.0 and time.time() - prev_draw_time > plot_rate:
            plotter.pause()
            prev_draw_time = time.time()

print("Bag finished")
plotter.stop()
