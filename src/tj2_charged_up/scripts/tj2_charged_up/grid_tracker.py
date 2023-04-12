import copy
from typing import List, Optional, Tuple
from threading import Lock
import rospy
import tf2_ros
import tf2_geometry_msgs
from networktables import NetworkTables

from std_msgs.msg import ColorRGBA
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import PoseStamped, Vector3, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray

from tj2_tools.robot_state import Pose2d
from tj2_tools.transforms import lookup_transform

from grid_zone_manager import Alliance, ColumnType, GridZone, GridZoneManager

class GridTracker:
    def __init__(self) -> None:
        self.node_name = "grid_tracker"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        grid_zones_path = rospy.get_param("~grid_zones_path", "grid_zones.csv")
        self.contact_threshold = rospy.get_param("~contact_threshold", 1.0)
        self.map_link = rospy.get_param("~global_frame_id", "map")
        self.base_link = rospy.get_param("~base_frame_id", "base_link")
        self.nt_host = rospy.get_param("~nt_host", "10.0.88.2")

        self.grid_zones = GridZoneManager.from_file(grid_zones_path)
        
        NetworkTables.initialize(server=self.nt_host)
        self.smart_dashboard_table = NetworkTables.getTable("SmartDashboard")
        self.grid_tracking_table = self.smart_dashboard_table.getSubTable("grid_tracking")
        self.grid_tracking_column_state = self.grid_tracking_table.getSubTable("column_state")
        
        self.status_entry = self.grid_tracking_table.getEntry("status")
        self.nearest_zone_entry = self.grid_tracking_table.getEntry("nearest_zone_distance")
        self.nearest_zone_serial_entry = self.grid_tracking_table.getEntry("nearest_zone_serial")
        self.nearest_zone_row_index = self.grid_tracking_table.getEntry("nearest_zone_row_index")
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.robot_tf_lock = Lock()
        self.global_tf = TransformStamped()
        self.global_tf.header.frame_id = self.map_link
        self.global_tf.child_frame_id = self.base_link
        self.global_tf.transform.rotation.w = 1.0

        self.detections_sub = rospy.Subscriber(
            "detections", Detection3DArray, self.detections_callback, queue_size=10
        )
        self.markers_pub = rospy.Publisher(
            "grid_tracking/markers", MarkerArray, queue_size=10
        )

    def update_global_tf(self) -> None:
        global_tf = lookup_transform(self.tf_buffer, self.map_link, self.base_link)
        if global_tf is not None:
            with self.robot_tf_lock:
                self.global_tf = global_tf

    def detections_callback(self, msg: Detection3DArray) -> None:
        assert msg.detections is not None
        if msg.header.frame_id == self.base_link:
            self.update_global_tf()
            global_tf = self.global_tf
        else:
            global_tf = lookup_transform(self.tf_buffer, self.map_link, msg.header.frame_id)
        if global_tf is None:
            return
        for detection in msg.detections:
            detection_pose = PoseStamped()
            detection_pose.header.frame_id = msg.header.frame_id
            detection_pose.pose = detection.bbox.center

            global_pose = tf2_geometry_msgs.do_transform_pose(detection_pose, self.global_tf)
            nearest_zone, distance = self.grid_zones.get_nearest(
                global_pose.pose.position.x,
                global_pose.pose.position.y,
                global_pose.pose.position.z
            )
            nearest_zone.filled = distance < self.contact_threshold
    
    def get_robot_pose(self) -> Optional[Pose2d]:
        zero_pose_base = PoseStamped()
        zero_pose_base.header.frame_id = self.base_link
        zero_pose_base.pose.orientation.w = 1.0

        global_pose = tf2_geometry_msgs.do_transform_pose(zero_pose_base, self.global_tf)
        return Pose2d.from_ros_pose(global_pose.pose)
    
    def report_state(self) -> None:
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            self.status_entry.setString("Invalid robot pose")
            return
        nearest_zone, distance = self.grid_zones.get_nearest(robot_pose.x, robot_pose.y, 0.0)
        
        self.nearest_zone_entry.setDouble(distance)
        self.nearest_zone_serial_entry.setString(self.get_serial(nearest_zone))

        row = self.grid_zones.get_row(nearest_zone.row)
        status = {ColumnType.HIGH: False, ColumnType.MID: False, ColumnType.LOW: False}
        row_index = nearest_zone.row
        for zone in row:
            status[zone.column] = zone.filled
        for column_type, filled in status.items():
            self.grid_tracking_column_state.getEntry(column_type.value).setBoolean(filled)
        self.nearest_zone_row_index.setDouble(row_index)
        self.status_entry.setString("OK")

    def publish_grid_state_markers(self) -> None:
        marker_array = MarkerArray()
        assert marker_array.markers is not None
        for zone in self.grid_zones.zones:
            markers = self.grid_zone_to_marker(zone)
            marker_array.markers.extend(markers)
        self.markers_pub.publish(marker_array)
    
    def grid_zone_to_marker(self, grid_zone: GridZone) -> List[Marker]:
        pose = PoseStamped()
        pose.header.frame_id = self.map_link
        pose.pose.position.x = grid_zone.x
        pose.pose.position.y = grid_zone.y
        pose.pose.position.z = grid_zone.z
        pose.pose.orientation.w = 1.0
        if grid_zone.filled:
            color = (0.0, 0.0, 0.0, 1.0)
        else:
            if grid_zone.alliance == Alliance.BLUE:
                color = (0.0, 0.0, 1.0, 1.0)
            else:
                color = (1.0, 0.0, 0.0, 1.0)
        sphere_marker = self.make_marker(self.get_serial(grid_zone), pose, color)
        sphere_marker.ns = "sphere" + sphere_marker.ns
        sphere_marker.type = Marker.SPHERE
    
        text_marker = self.make_marker(self.get_serial(grid_zone), pose, color)
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.ns = "text" + text_marker.ns
        text_marker.text = f"{grid_zone.column}-{grid_zone.row} {grid_zone.type}"
        text_marker.scale.x = 0.0
        text_marker.scale.y = 0.0
        text_marker.color = ColorRGBA(
            r=1.0,
            g=1.0,
            b=1.0,
            a=1.0,
        )
        
        return [sphere_marker, text_marker]
    
    def make_marker(self, name: str, pose: PoseStamped, color: Tuple[float, float, float, float]):
        marker = Marker()
        marker.action = Marker.ADD
        marker.pose = copy.deepcopy(pose.pose)
        marker.header = pose.header
        marker.lifetime = rospy.Duration(1.0)
        marker.ns = name
        marker.id = 0  # all waypoint names should be unique

        scale_vector = Vector3()
        scale_vector.x = self.contact_threshold
        scale_vector.y = self.contact_threshold
        scale_vector.z = self.contact_threshold
        marker.scale = scale_vector
        marker.color = ColorRGBA(
            r=color[0],
            g=color[1],
            b=color[2],
            a=color[3],
        )

        return marker

    @staticmethod
    def get_serial(zone: GridZone) -> str:
        return f"{zone.alliance}-{zone.column}-{zone.row}"
    
    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.update_global_tf()
            self.report_state()
            self.publish_grid_state_markers()
            rate.sleep()


if __name__ == '__main__':
    node = GridTracker()
    node.run()
