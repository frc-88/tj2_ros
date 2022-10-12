from tj2_tools.rosbag_to_file.json_loader import iter_bag, get_key, header_to_stamp, yaw_from_quat, read_pkl


def read_states(path):
    data = []
    print("Creating pickle from %s" % path)

    for timestamp, topic, msg in iter_bag(path):
        if topic == "/tj2/swerve_modules":
            frame_id = get_key(msg, "header.frame_id")
            index = int(frame_id.split("_")[-1])
            if index >= 4 or index < 0:
                continue

            timestamp *= 1E-9
            
            row = {
                "time": timestamp,
                "wheel_velocity": get_key(msg, "wheel_velocity"),
                "azimuth_velocity": get_key(msg, "azimuth_velocity"),
                "azimuth": get_key(msg, "azimuth"),
                "wheel_velocity_ref": get_key(msg, "wheel_velocity_reference"),
                "azimuth_velocity_ref": get_key(msg, "azimuth_velocity_reference"),
                "azimuth_ref": get_key(msg, "azimuth_reference"),
                "hi_voltage_ref": get_key(msg, "hi_voltage_reference"),
                "lo_voltage_ref": get_key(msg, "lo_voltage_reference"),
                "hi_voltage": get_key(msg, "hi_voltage"),
                "lo_voltage": get_key(msg, "lo_voltage"),
                "hi_current": get_key(msg, "hi_current"),
                "lo_current": get_key(msg, "lo_current"),
            }

            while len(data) <= index:
                data.append([])
            module_data = data[index]

            module_data.append(row)

    return data


def get_states(path, repickle=False):
    return read_pkl(path, read_states, repickle)
