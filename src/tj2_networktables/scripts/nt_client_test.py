import time
from networktables import NetworkTables
from geometry_msgs.msg import PoseStamped
from tj2_tools.networktables.ros_conversions import ros_msg_to_base64_json, base64_json_to_ros_msg, convert_to_nt_topic


NetworkTables.startClient(("0.0.0.0", 5800))
NetworkTables.setUpdateRate(0.02)

ros_to_nt_subtable = NetworkTables.getTable('ros_to_nt')
nt_to_ros_subtable = NetworkTables.getTable('nt_to_ros')

ros_to_nt_subtable.getEntry(convert_to_nt_topic("/something/string")).setString("")


x = 0.0
while True:
    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.pose.position.x = x
    msg.pose.position.y = 2.0
    msg.pose.position.z = 3.0
    x += 1.0
    nt_to_ros_subtable.getEntry(convert_to_nt_topic("/something/pose")).setString(ros_msg_to_base64_json(msg))
    ros_msg, ros_msg_type = base64_json_to_ros_msg(ros_to_nt_subtable.getString(convert_to_nt_topic("/something/string"), ""))
    print("recv:", ros_msg)

    time.sleep(0.02)
