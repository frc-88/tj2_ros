#!/usr/bin/env python3
import rospy
import tf2_ros
from typing import List
from geometry_msgs.msg import TransformStamped


def main():
    rospy.init_node("static_transforms")

    transforms_parameter = rospy.get_param("~transforms", {})
    tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

    static_transforms: List[TransformStamped] = []

    for definition in transforms_parameter:
        parent = definition["parent"]
        child = definition["child"]
        translate = definition["translate"]
        rotate = definition["rotate"]

        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.translation.x = translate[0]
        transform.transform.translation.y = translate[1]
        transform.transform.translation.z = translate[2]
        transform.transform.rotation.x = rotate[0]
        transform.transform.rotation.y = rotate[1]
        transform.transform.rotation.z = rotate[2]
        transform.transform.rotation.w = rotate[3]
        static_transforms.append(transform)

    tf_broadcaster.sendTransform(static_transforms)
    rospy.spin()
