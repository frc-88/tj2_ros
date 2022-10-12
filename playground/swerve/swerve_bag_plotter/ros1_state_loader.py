from tj2_tools.rosbag_to_file.json_loader import iter_bag, get_key, header_to_stamp, yaw_from_quat, read_pkl


def read_states(path):
    data = []
    print("Creating pickle from %s" % path)

    for timestamp, topic, msg in iter_bag(path):
        if topic == "/tj2/swerve_modules":
            index = get_key(msg, "module_index")
            index = int(index)
            if index >= 4 or index < 0:
                continue

            row = {
                "time": timestamp,
                "wheel_velocity": get_key(msg, "wheel_velocity"),
                "azimuth_velocity": get_key(msg, "azimuth_velocity"),
                "azimuth": get_key(msg, "azimuth_position"),
                "wheel_velocity_ref": get_key(msg, "wheel_velocity_ref"),
                "azimuth_velocity_ref": get_key(msg, "azimuth_velocity_ref"),
                "azimuth_ref": get_key(msg, "azimuth_position_ref"),
                "hi_voltage_ref": get_key(msg, "motor_hi_1.voltage_ref"),
                "hi_voltage": get_key(msg, "motor_hi_1.voltage"),
                "hi_velocity": get_key(msg, "motor_hi_1.velocity"),
                "lo_voltage_ref": get_key(msg, "motor_lo_0.voltage_ref"),
                "lo_voltage": get_key(msg, "motor_lo_0.voltage"),
                "lo_velocity": get_key(msg, "motor_lo_0.velocity"),
            }

            while len(data) <= index:
                data.append([])
            module_data = data[index]

            module_data.append(row)

    return data


def get_states(path, repickle=False):
    return read_pkl(path, read_states, repickle)
