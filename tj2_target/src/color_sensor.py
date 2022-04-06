#!/usr/bin/python3
import math
import rospy

from std_msgs.msg import ColorRGBA
from std_msgs.msg import Int16
from std_msgs.msg import String

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3

from tj2_networktables.msg import NTEntry

from tj2_tools.rev_color_sensor import ColorSensorV3


class ColorSensorNode:
    def __init__(self):
        self.name = "color_sensor"
        rospy.init_node(
            self.name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )

        self.loop_rate = rospy.get_param("loop_rate", 15.0)
        self.color_match_table = rospy.get_param("color_match_table", None)
        if self.color_match_table is None:
            self.color_match_table = {}
        self.color_confidence_threshold = rospy.get_param("color_confidence_threshold", 0.5)
        self.marker_frame_location = rospy.get_param("marker_frame_location", "")
        self.marker_size = rospy.get_param("marker_size", 0.5)

        self.nt_pub = rospy.Publisher("nt_passthrough", NTEntry, queue_size=10)
        self.color_pub = rospy.Publisher("color_sensor/raw", ColorRGBA, queue_size=10)
        self.proximity_pub = rospy.Publisher("color_sensor/proximity", Int16, queue_size=10)
        self.color_match_pub = rospy.Publisher("color_sensor/match", String, queue_size=10)
        self.color_marker_pub = rospy.Publisher("color_sensor/marker", MarkerArray, queue_size=10)

        self.color_sensor = ColorSensorV3()

        rospy.loginfo("%s init complete" % self.name)

    @staticmethod
    def get_color_distance(rgb1, rgb2=None):
        if rgb2 is None:
            rgb2 = [0.0, 0.0, 0.0]
        dr = rgb1[0] - rgb2[0]
        dg = rgb1[1] - rgb2[1]
        db = rgb1[2] - rgb2[2]
        return math.sqrt(dr * dr + dg * dg + db * db)

    def match_color(self, sensed_rgb, confidence_threshold=0.5):
        # inspired by REV robotics code:
        # https://github.com/REVrobotics/Color-Sensor-v3-Examples/blob/master/Java/Color%20Match/src/main/java/frc/robot/Robot.java
        # https://docs.revrobotics.com/color-sensor/application-examples
        # sensed_rgb has channels from 0.0..1.0
        magnitude = self.get_color_distance(sensed_rgb)
        min_dist = 1.0
        match_name = ""
        for name, rgb in self.color_match_table.items():
            distance = self.get_color_distance(rgb, sensed_rgb)
            if distance < min_dist:
                min_dist = distance
                match_name = name
        confidence = 1.0 - (min_dist / magnitude)
        if confidence > confidence_threshold:
            return match_name
        else:
            return ""
    
    def run(self):
        clock = rospy.Rate(self.loop_rate)
        
        while not rospy.is_shutdown():
            color = self.color_sensor.getColor()
            color_msg = ColorRGBA(color.red, color.green, color.blue, 1.0)
            self.color_pub.publish(color_msg)

            proximity = self.color_sensor.getProximity()
            proximity_msg = Int16(proximity)
            self.proximity_pub.publish(proximity_msg)

            color_match = self.match_color([color.red, color.green, color.blue], self.color_confidence_threshold)
            self.color_match_pub.publish(String(color_match))

            self.publish_marker(color_match, color_msg)

            self.nt_pub.publish(self.make_entry("color_sensor/r", color.red))
            self.nt_pub.publish(self.make_entry("color_sensor/g", color.green))
            self.nt_pub.publish(self.make_entry("color_sensor/b", color.blue))
            self.nt_pub.publish(self.make_entry("color_sensor/proximity", proximity))
            self.nt_pub.publish(self.make_entry("color_sensor/match", color_match))

            clock.sleep()

    def publish_marker(self, match_name, color_msg):
        if len(self.marker_frame_location) == 0:
            return
        sphere_marker = self.make_marker(color_msg)
        text_marker = self.make_marker(color_msg)

        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.ns = "text" + text_marker.ns
        text_marker.text = match_name if len(match_name > 0) else "<no match>"
        text_marker.scale.x = 0.0
        text_marker.scale.y = 0.0

        sphere_marker.type = Marker.SPHERE
        sphere_marker.ns = "sphere" + sphere_marker.ns
        sphere_marker.color.a = 0.75

        markers = MarkerArray()
        markers.markers.append(sphere_marker)
        markers.markers.append(text_marker)
        self.color_marker_pub.append(markers)

    def make_marker(self, color_msg):
        pose = Pose()
        pose.orientation.w = 1.0
        marker = Marker()
        marker.action = Marker.ADD
        marker.pose = pose
        marker.header.frame_id = self.map_frame
        marker.lifetime = rospy.Duration(1.0)  # seconds
        marker.ns = "color_sensor"
        marker.id = 0

        scale_vector = Vector3()
        scale_vector.x = self.marker_size
        scale_vector.y = self.marker_size
        scale_vector.z = self.marker_size
        marker.scale = scale_vector
        marker.color = color_msg

if __name__ == "__main__":
    node = ColorSensorNode()
    try:
        node.run()

    except rospy.ROSInterruptException:
        pass

    finally:
        rospy.loginfo("Exiting %s node" % node.name)
