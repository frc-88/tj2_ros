import csv
from ros2_state_loader import get_states

data = get_states("data/diffyjr_2022-06-09-22-42-13_0.json")


length = min([len(data[module_index]) for module_index in range(len(data))])


table = []
for index in range(length):
    table.append([])
    for module_index in range(len(data)):
        module_data = data[module_index][index]
        wheel_velocity = module_data["wheel_velocity"]
        azimuth_velocity = module_data["azimuth_velocity"]

        table[-1].append(wheel_velocity)
        table[-1].append(azimuth_velocity)

with open("data/perimeter.csv", 'w') as file:
    header = []
    for module_index in range(len(data)):
        header.append("wheel_velocity_" + str(module_index))
        header.append("azimuth_velocity_" + str(module_index))
    writer = csv.writer(file)
    writer.writerow(header)
    for row in table:
        writer.writerow(row)
