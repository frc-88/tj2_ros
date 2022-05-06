from tj2_tools.rosbag_to_file.json_loader import iter_bag, get_key, header_to_stamp, yaw_from_quat, read_pkl

CLASS_NAMES = [
    "power_cell"
]

robot_log = []
cargo_log = []

def get_label(obj_id):
    class_index = obj_id & 0xffff
    class_count = obj_id >> 16

    label = CLASS_NAMES[class_index]
    return label, class_count

def print_msg(msg, msg_key):
    print(msg_key, ': ', get_key(msg, msg_key))

def read_states(path):
    states = []
    print("Creating pickle from %s" % path)

    for timestamp, topic, msg in iter_bag(path):
        if topic == "/tj2/cmd_vel":
            None
            #print(msg)
            # state = Simple3DState()
            # state.type = "cmd_vel"
            # state.stamp = timestamp
            # state.vx = get_key(msg, "linear.x")
            # state.vy = get_key(msg, "linear.y")
            # state.vt = get_key(msg, "angular.z")
            # states.append(state)

        elif topic == "/tj2/odom":
            #None
            #print(msg)
            #continue
            print('odom: ===================================================')
            print_msg(msg, "header.stamp")
            print_msg(msg, "pose.pose.position.x")
            print_msg(msg, "pose.pose.position.y")
            print_msg(msg, "pose.pose.position.z")
            print_msg(msg, "pose.pose.orientation")
            print_msg(msg, "twist.twist.linear.x")
            print_msg(msg, "twist.twist.linear.y")
            print_msg(msg, "twist.twist.linear.z")
            print_msg(msg, "twist.twist.angular.z")
            log = dict()
            log['time'] = get_key(msg, 'header.stamp.secs') + get_key(msg, 'header.stamp.nsecs')*0.000000001
            log['position'] = get_key(msg, 'pose.pose')
            log['twist'] = get_key(msg, 'twist.twist')
            robot_log.append(log)

            pos = get_key(msg, 'pose.pose')
            tst = get_key(msg, 'twist.twist')


            # state = Simple3DState()
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
            #print(msg)
            print('detections: ***********************************************************************************************')
            print_msg(msg, "header.stamp")
            log = dict()
            log['time'] = get_key(msg, 'detections.0.header.stamp.secs') + get_key(msg, 'detections.0.header.stamp.nsecs') * 0.000000001
            #print_msg(msg, "detections")
            detections = get_key(msg, "detections")
            frame_detections = []
            for index, detection in enumerate(detections.values()):
                object_id = get_key(detection, "results.0.id")
                label, count = get_label(object_id)
                print('label: ', label, '  count: ', count, '---------')
                print_msg(detection, "results.0.pose.pose.position.x")
                print_msg(detection, "results.0.pose.pose.position.y")
                print_msg(detection, "results.0.pose.pose.position.z")
                cargo = dict()
                cargo['label'], cargo['count'] = label, count
                cargo['pos'] = get_key(detection, 'results.0.pose.pose.position')
                frame_detections.append(cargo)
            log['detections'] = frame_detections
            cargo_log.append(log)


            '''
            stamp = header_to_stamp(get_key(msg, "header.stamp"))
            detections = get_key(msg, "detections")
            for index, detection in enumerate(detections.values()):
                object_id = get_key(detection, "results.0.id")
                label, count = get_label(object_id)

                state = Simple3DState()
                state.type = label
                state.stamp = stamp
                state.x = get_key(detection, "results.0.pose.pose.position.x")
                state.y = get_key(detection, "results.0.pose.pose.position.y")
                state.z = get_key(detection, "results.0.pose.pose.position.z")

                states.append(state)
            '''


    return robot_log, cargo_log


def get_states(path, repickle=False):
    return read_pkl(path, read_states, repickle)
