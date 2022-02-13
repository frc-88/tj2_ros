from tj2_tools.rosbag_to_file.json_loader import iter_bag, get_key, header_to_stamp, yaw_from_quat, read_pkl

CLASS_NAMES = [
    "power_cell"
]


def get_label(obj_id):
    class_index = obj_id & 0xffff
    class_count = obj_id >> 16

    label = CLASS_NAMES[class_index]
    return label, class_count


def read_states(path):
    states = []
    print("Creating pickle from %s" % path)

    for timestamp, topic, msg in iter_bag(path):
        if topic == "/tj2/cmd_vel":
            #None
            print(msg)
            # state = FilterState()
            # state.type = "cmd_vel"
            # state.stamp = timestamp
            # state.vx = get_key(msg, "linear.x")
            # state.vy = get_key(msg, "linear.y")
            # state.vt = get_key(msg, "angular.z")
            # states.append(state)

        elif topic == "/tj2/odom":
            #None
            print(msg)
            # state = FilterState()
            # state.type = "odom"
            # state.stamp = header_to_stamp(get_key(msg, "header.stamp"))
            # state.x = get_key(msg, "pose.pose.position.x")
            # state.y = get_key(msg, "pose.pose.position.y")
            # state.z = get_key(msg, "pose.pose.position.z")
            # state.theta = yaw_from_quat(get_key(msg, "pose.pose.orientation"))
            #
            # state.vx = get_key(msg, "twist.twist.linear.x")
            # state.vy = get_key(msg, "twist.twist.linear.y")
            # state.vz = get_key(msg, "twist.twist.linear.z")
            # state.vt = get_key(msg, "twist.twist.angular.z")
            #
            # states.append(state)

        elif topic == "/tj2/powercell/detections":
            None
            print(msg)
            '''            stamp = header_to_stamp(get_key(msg, "header.stamp"))
            detections = get_key(msg, "detections")
            for index, detection in enumerate(detections.values()):
                object_id = get_key(detection, "results.0.id")
                label, count = get_label(object_id)

                state = FilterState()
                state.type = label
                state.stamp = stamp
                state.x = get_key(detection, "results.0.pose.pose.position.x")
                state.y = get_key(detection, "results.0.pose.pose.position.y")
                state.z = get_key(detection, "results.0.pose.pose.position.z")

                states.append(state)
            '''


    return states


def get_states(path, repickle=False):
    return read_pkl(path, read_states, repickle)
