import os
import csv
import rospkg


def meters_to_feet(meters):
    return meters * 3.28084

rospack = rospkg.RosPack()
table_path = os.path.join(rospack.get_path("tj2_target"), "config", "time_of_flight.csv")

hood_down = []
hood_up = []
with open(table_path) as file:
    reader = csv.reader(file)
    header = next(reader)

    for row in reader:
        hood_state = row[header.index("hood")]
        data = list(map(float, row[1:]))
        assert len(data) == 2
        data[0] = meters_to_feet(data[0])
        if hood_state == "down":
            hood_down.append(data)
        elif hood_state == "up":
            hood_up.append(data)
        else:
            raise ValueError("Invalid hood state: %s" % row)

java_code_template = """private final ValueInterpolator m_tofInterpolator{hood_state} = new ValueInterpolator(
{data}
);
"""

def get_data_code(hood_data):
    data_code = ""
    for index, row in enumerate(hood_data):
        distance_feet = row[0]
        time_seconds = row[1]
        data_code += f"    new ValueInterpolator.ValuePair({distance_feet}, {time_seconds})"
        if index < len(hood_data) - 1:
            data_code += ","
            data_code += "\n"
    return data_code

hood_up_code = java_code_template.format(hood_state="HoodUp", data=get_data_code(hood_up))
hood_down_code = java_code_template.format(hood_state="HoodDown", data=get_data_code(hood_down))

print("// TOF table for hood up position (distance feet -> time of flight seconds):")
print(hood_up_code)
print("// TOF table for hood down position (distance feet -> time of flight seconds):")
print(hood_down_code)
