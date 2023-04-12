#!/usr/bin/env python3
import copy
import time
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
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import CameraInfo
from nav_msgs.msg import Odometry

from tj2_tools.robot_state import Pose2d, Velocity
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
        self.no_fill_timeout = rospy.get_param("~no_fill_timeout", 1.0)
        self.linear_velocity_threshold = rospy.get_param("~linear_velocity_threshold", 1.0)
        self.angular_velocity_threshold = rospy.get_param("~angular_velocity_threshold", 1.0)
        self.map_link = rospy.get_param("~global_frame_id", "map")
        self.base_link = rospy.get_param("~base_frame_id", "base_link")
        self.nt_host = rospy.get_param("~nt_host", "10.0.88.2")

        self.grid_zones = GridZoneManager.from_file(grid_zones_path)
        self.camera_model: Optional[PinholeCameraModel] = None
        
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
        self.zones_lock = Lock()
        self.global_tf = TransformStamped()
        self.global_tf.header.frame_id = self.map_link
        self.global_tf.child_frame_id = self.base_link
        self.global_tf.transform.rotation.w = 1.0
        self.robot_velocity = Velocity()

        self.markers_pub = rospy.Publisher(
            "grid_tracking/markers", MarkerArray, queue_size=10
        )
        self.detections_sub = rospy.Subscriber(
            "detections", Detection3DArray, self.detections_callback, queue_size=10
        )
        self.odom_sub = rospy.Subscriber(
            "odom", Odometry, self.odom_callback, queue_size=10
        )
        self.camera_info_sub = rospy.Subscriber("camera_info", CameraInfo, self.info_callback, queue_size=1)
        rospy.loginfo("Grid tracker initialized")

    def update_global_tf(self) -> None:
        global_tf = lookup_transform(self.tf_buffer, self.map_link, self.base_link)
        if global_tf is not None:
            with self.robot_tf_lock:
                self.global_tf = global_tf

    def info_callback(self, msg: CameraInfo) -> None:
        rospy.loginfo("Got camera model")
        self.camera_info_sub.unregister()
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(msg)

    def odom_callback(self, msg: Odometry) -> None:
        self.robot_velocity = Velocity.from_ros_twist(msg.twist.twist)

    def detections_callback(self, msg: Detection3DArray) -> None:
        if self.robot_velocity.magnitude() > self.linear_velocity_threshold:
            return
        if abs(self.robot_velocity.theta) > self.angular_velocity_threshold:
            return 
        assert msg.detections is not None
        if msg.header.frame_id == self.base_link:
            self.update_global_tf()
            global_tf = self.global_tf
        else:
            global_tf = lookup_transform(self.tf_buffer, self.map_link, msg.header.frame_id)
        if global_tf is None:
            return
        if self.camera_model is not None:
            reverse_tf = lookup_transform(self.tf_buffer, self.camera_model.tfFrame(), self.map_link)
        else:
            reverse_tf = None

        with self.zones_lock:
            if reverse_tf is not None:
                for zone in self.grid_zones.zones:
                    zone.set_in_view(self.is_zone_in_view(reverse_tf, zone))
                    now = time.monotonic()
                    did_fill_timeout = now - zone.filled_time > self.no_fill_timeout
                    did_not_in_view_timeout = now - zone.not_in_view_time > self.no_fill_timeout
                    if zone.in_view and did_fill_timeout and did_not_in_view_timeout:
                        zone.filled = False
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
                nearest_zone.set_filled(distance < self.contact_threshold)
    
    def is_zone_in_view(self, reverse_tf: Optional[TransformStamped], zone: GridZone) -> bool:
        if reverse_tf is None or self.camera_model is None:
            return False
        grid_pose = PoseStamped()
        grid_pose.header.frame_id = self.map_link
        grid_pose.pose.position.x = zone.x
        grid_pose.pose.position.y = zone.y
        grid_pose.pose.position.z = zone.z
        camera_relative_grid = tf2_geometry_msgs.do_transform_pose(grid_pose, reverse_tf)
        if camera_relative_grid.pose.position.z < 0.0:
            return False

        px = self.camera_model.project3dToPixel((
            camera_relative_grid.pose.position.x,
            camera_relative_grid.pose.position.y,
            camera_relative_grid.pose.position.z,
        ))
        return 0 < px[0] < self.camera_model.width and 0 < px[1] < self.camera_model.height  # type: ignore
    
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
        with self.zones_lock:
            nearest_zone, distance = self.grid_zones.get_nearest(robot_pose.x, robot_pose.y, 0.0)
            self.nearest_zone_entry.setDouble(distance)
            self.nearest_zone_serial_entry.setString(self.get_serial(nearest_zone))

            row = self.grid_zones.get_row(nearest_zone.alliance, nearest_zone.row)
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
        with self.zones_lock:
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
            color = (0.0, 0.0, 0.0, 0.95)
        else:
            alpha = 0.95 if grid_zone.in_view else 0.25
            if grid_zone.alliance == Alliance.BLUE:
                color = (0.0, 0.0, 1.0, alpha)
            else:
                color = (1.0, 0.0, 0.0, alpha)
        sphere_marker = self.make_marker(self.get_serial(grid_zone), pose, color)
        sphere_marker.ns = "sphere" + sphere_marker.ns
        sphere_marker.type = Marker.SPHERE
    
        text_marker = self.make_marker(self.get_serial(grid_zone), pose, color)
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.ns = "text" + text_marker.ns
        text_marker.text = f"{grid_zone.column.value}-{grid_zone.row} {grid_zone.type.value}"
        text_marker.scale.x = 0.0
        text_marker.scale.y = 0.0
        text_marker.scale.z = 0.15
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
        scale_vector.x = self.contact_threshold * 2
        scale_vector.y = self.contact_threshold * 2
        scale_vector.z = self.contact_threshold * 2
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
        return f"{zone.alliance.value}-{zone.column.value}-{zone.row}"
    
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
