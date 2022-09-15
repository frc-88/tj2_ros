import time
import numpy as np
from module_velocity_plotter import ModuleVelocityPlotter
from ros1_state_loader import get_states

# field relative rotations
data = get_states("data/diffyjr_2022-09-15-10-02-04.json")


plotter = ModuleVelocityPlotter(4.76, 47.37, 2 * np.pi, history=None, playback_real_time=False)
plotter.draw_perimeter_from_file("data/perimeter.csv")
plotter.init()

sim_start_t = data[0][0]["time"]

length = min([len(data[module_index]) for module_index in range(len(data))])

for index in range(length):
    sim_time = data[0][index]["time"]
    sim_duration = sim_time - sim_start_t
    for module_index in range(len(data)):
        module_data = data[module_index][index]
        plotter.draw(sim_duration, module_index, module_data)

print("Bag finished")
plotter.stop()
