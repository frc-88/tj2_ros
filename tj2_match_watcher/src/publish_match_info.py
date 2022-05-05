#!/usr/bin/python3
import os
import rospy

from tj2_networktables.msg import NTEntry

from vision_msgs.msg import Detection3DArray
from vision_msgs.msg import Detection3D

from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

from std_msgs.msg import ColorRGBA

from tj2_tools.yolo.utils import get_label, read_class_names


class PublishMatchInfo:
    def __init__(self):
        self.name = "publish_match_info"
        self.nt_table = {}
        rospy.init_node(
            self.name
        )

        self.class_names_path = rospy.get_param("~class_names_path", "./objects.names")
        self.marker_persistance = rospy.Duration(rospy.get_param("~marker_persistance", 0.08))

        self.nt_sub = rospy.Subscriber("/tj2/smart_dashboard", NTEntry, self.nt_callback)
        self.detections_sub = rospy.Subscriber("/tj2/detections", Detection3DArray, self.detections_callback)
        
        self.markers_pub = rospy.Publisher("/tj2/markers", MarkerArray, queue_size=25)

        self.class_names = read_class_names(self.class_names_path)

        self.marker_colors = {
            "cargo_red": ColorRGBA(1.0, 0.0, 0.0, 1.0),
            "cargo_blue": ColorRGBA(0.0, 0.0, 1.0, 1.0),
        }

    def nt_callback(self, msg):
        self.nt_table[msg.path] = msg.value

    def detections_callback(self, msg):
        markers = MarkerArray()
        for detection_3d_msg in msg.detections:
            color = self.marker_colors[self.get_detection_label(detection_3d_msg)[0]]
            self.add_detection_to_marker_array(markers, detection_3d_msg, color)
        self.markers_pub.publish(markers)

    def get_detection_label(self, detection_3d_msg):
        return get_label(self.class_names, detection_3d_msg.results[0].id)

    def make_marker(self, detection_3d_msg, color):
        marker = Marker()
        marker.action = Marker.ADD
        marker.pose = detection_3d_msg.results[0].pose.pose
        marker.header = detection_3d_msg.header
        marker.lifetime = self.marker_persistance
        label, count = self.get_detection_label(detection_3d_msg)
        marker.ns = label
        marker.id = count

        marker.scale.x = detection_3d_msg.bbox.size.x
        marker.scale.y = detection_3d_msg.bbox.size.y
        marker.scale.z = detection_3d_msg.bbox.size.z
        marker.color = color

        return marker

    def add_detection_to_marker_array(self, marker_array, detection_3d_msg, color):
        confidence = detection_3d_msg.results[0].score
        if confidence < 0.9:
            return

        label, count = self.get_detection_label(detection_3d_msg)

        sphere_marker = self.make_marker(detection_3d_msg, color)
        text_marker = self.make_marker(detection_3d_msg, color)

        sphere_marker.type = Marker.SPHERE
        sphere_marker.ns = "sphere_" + sphere_marker.ns

        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.ns = "text_" + sphere_marker.ns
        text_marker.text = "%s_%s|%0.1f" % (label, count, confidence * 100)
        text_marker.scale.z = min(text_marker.scale.x, text_marker.scale.y)
        text_marker.scale.x = 0.0
        text_marker.scale.y = 0.0

        marker_array.markers.append(sphere_marker)
        marker_array.markers.append(text_marker)

    def run(self):
        rospy.spin()
        # while True:
        #     if rospy.is_shutdown():
        #         break
        #     print("---")
        #     for path, value in self.nt_table.items():
        #         print("%s:\t%s" % (path, value))
        #     print()
        #     rospy.sleep(0.25)


def main():
    node = PublishMatchInfo()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting %s node" % node.name)


if __name__ == '__main__':
    main()
