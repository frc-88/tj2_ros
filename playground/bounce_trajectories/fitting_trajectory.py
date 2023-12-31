
import numpy as np
from tj2_tools.rosbag_to_file.json_loader import iter_bag, get_key, header_to_stamp, yaw_from_quat, read_pkl
'''
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
    msgs = []
    print("Creating pickle from %s" % path)

    for timestamp, topic, msg in iter_bag(path):
        if topic == "/tj2/cmd_vel":
            None
        elif topic == "/tj2/odom" or topic == "/tj2/powercell/detections":
            msgs.append(msg)

    return msgs


def get_states(path, repickle=False):
    return read_pkl(path, read_states, repickle)

repickle = True

#msgs = get_states("data/detections_2022-02-03-23-59-49.json", repickle)
#msgs = get_states("data/detections_2022-02-04-00-02-36.json", repickle)
#msgs = get_states("C:/FILES/TJ square/prediction-data-2/detections_2022-02-09-10-58-15.json", repickle)
msgs = get_states("C:/FILES/TJ square/prediction-data-2/detections_2022-02-09-10-57-39.json", repickle)

odom_stamps = []
detection_stamps = []
for i, msg in enumerate(msgs):
    if 'twist' not in msg.keys():
        time = get_key(msg, 'detections.0.header.stamp.secs') + get_key(msg, 'detections.0.header.stamp.nsecs') * 0.000000001
        detection_stamps.append(time)
    else:
        time = get_key(msg, 'header.stamp.secs') + get_key(msg, 'header.stamp.nsecs') * 0.000000001
        odom_stamps.append(time)

print('Intervals for odom ======================================================================')
for i in range(len(odom_stamps)-1):
    interval = odom_stamps[i+1] - odom_stamps[i]
    print(interval)

print('Intervals for detection ================================================================')
for i in range(len(detection_stamps)-1):
    interval = detection_stamps[i+1] - detection_stamps[i]
    print(interval)
'''
def predict(msgs):
    confident = 0
    print(len(msgs))
    print('-------------------------------------------------------------------')
    rxs, rys = [], [] # robot xs and ys
    cxs, cys = [], [] # cargo xs and ys
    track_img = np.zeros((200,200)).astype(np.uint8)
    pairs = []
    for i, msg in enumerate(msgs):
        if 'detections' in msg.keys():
            detect_time = get_key(msg, 'detections.0.header.stamp.secs') + get_key(msg, 'detections.0.header.stamp.nsecs') * 0.000000001
            mmin = 1000000
            odom_idx = 0
            for j in range(i):
                if 'twist' in msgs[j].keys():
                    odom_time = get_key(msgs[j], 'header.stamp.secs') + get_key(msgs[j], 'header.stamp.nsecs')*0.000000001
                    delta = abs(detect_time - odom_time)
                    if mmin > delta:
                        mmin = delta
                        odom_idx = j
            if mmin < 0.02:
                pairs.append([i, odom_idx])

    print(pairs)
    if len(pairs) < 6:
        return []

    org_x, org_y = 0, 0
    org_angle = 0
    for msg in msgs:
        if 'twist' in msg.keys():
            org_x = get_key(msg, "pose.pose.position.x")
            org_y = get_key(msg, "pose.pose.position.y")
            quaternion = get_key(msg, 'pose.pose.orientation').values()
            q3, q0, q1, q2 = quaternion
            org_angle = np.arctan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3))  # in rad
            break

    # https://www.cnblogs.com/jins-note/p/9512719.html
    # quaternion: line segment whose length is 1, (x, y, z), an angle theta
    # the corresponding quaternion is q = ((x, y, z)*sin(theta/2), cos(theta/2))
    # 四元数到欧拉角：https://www.cnblogs.com/21207-ihome/p/6894128.html
    ccxs, ccys = [], []
    stmps = []
    for pair in pairs:
        cargo_idx, robot_idx = pair
        cx = get_key(msgs[cargo_idx], 'detections.0.results.0.pose.pose.position.x')
        cy = get_key(msgs[cargo_idx], 'detections.0.results.0.pose.pose.position.y')
        rx = get_key(msgs[robot_idx], 'pose.pose.position.x')
        ry = get_key(msgs[robot_idx], 'pose.pose.position.y')
        time = get_key(msgs[cargo_idx], 'detections.0.header.stamp.secs') + get_key(msgs[cargo_idx], 'detections.0.header.stamp.nsecs') * 0.000000001
        stmps.append(time)
        quaternion = get_key(msgs[robot_idx], 'pose.pose.orientation').values()
        q3, q0, q1, q2 = quaternion
        angle = np.arctan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3)) # in rad
        delta_angle = angle - org_angle
        angle = delta_angle if delta_angle > 0 else delta_angle+2*np.pi
        cos, sin = np.cos(-angle), np.sin(-angle) # rotate in the opposite direction
        ccx, ccy = cx*cos+cy*sin, cy*cos+cx*sin
        ccx, ccy = rx-org_x+ccx, ry-org_y+ccy
        ccxs.append(ccx)
        ccys.append(ccy)
        print(angle)

    fl = 5 # fitting length
    vxs, vys = [], []
    for i in range(fl):
        delta_x = ccxs[-i-1] - ccxs[-i-2]
        delta_y = ccys[-i-1] - ccys[-i-2]
        time = stmps[-i-1] - stmps[-i-2] + 0.0000001
        vx, vy = delta_x/time, delta_y/time
        vxs.append(vx)
        vys.append(vy)
    sumx, sumy = sum(vxs), sum(vys)
    sumxx, sumyy = sum([vx*vx for vx in vxs]), sum([vy*vy for vy in vys])
    varx, vary = sumxx/fl - sumx*sumx/(fl*fl), sumyy/fl - sumy*sumy/(fl*fl)
    sigma_vx, sigma_vy = np.sqrt(varx), np.sqrt(vary)
    sigma_th = 1
    if sigma_vx > sigma_th or sigma_vy > sigma_th:
        return []

    sumx = sumy = sumxx = sumyy = sumxy = 0
    for x, y in list(zip(ccxs, ccys))[-fl:]:
        sumx += x
        sumy += y
        sumxx += x*x
        sumyy += y*y
        sumxy += x*y

    denox, denoy = fl * sumxx - sumx * sumx, fl * sumyy - sumy * sumy

    if denox > denoy:
        k = (fl * sumxy - sumx * sumy) / denox if denox != 0 else 1000
        b = (sumy/fl) - k*(sumx/fl)
        vx = (ccxs[-1] - ccxs[-4]) / (stmps[-1]-stmps[-4])
        vy = vx*k
        vy = vy if vy*(ccys[-1] - ccys[-4])>0 else -vy
    else:
        k = denoy / (fl * sumxy - sumx * sumy) if (fl * sumxy - sumx * sumy) != 0 else 1000
        k = 1/k
        b = (sumx/fl) - k*(sumy/fl)
        vy = (ccys[-1]-ccys[-4]) / (stmps[-1]-stmps[-4])
        vx = vy*k
        vx = vx if vx*(ccxs[-1]-ccxs[-4])>0 else -vx

    return vx, vy, ccxs[-1], ccys[-1]
'''
BUF_LEN = 1*(55+20) # roughly 1 sec
for i, msg in enumerate(msgs):
    if 'detections' in msg.keys():
        msg_buf = msgs[i-BUF_LEN:i+1]
        if len(msg_buf) < BUF_LEN:
            continue
        predict(msg_buf)


print('Get coordinates. ')
'''