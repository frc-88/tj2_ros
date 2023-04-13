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
    """
    GridTracker class is used to track FIRST robotics competition game pieces in the grid zones
    in the 2023 game "Charged Up!"
    """

    def __init__(self) -> None:
        # Initialize the node, setting its name to "grid_tracker"
        self.node_name = "grid_tracker"
        rospy.init_node(self.node_name)

        # Load the path to the CSV file containing grid zone information
        grid_zones_path = rospy.get_param("~grid_zones_path", "grid_zones.csv")

        # Load the contact_threshold parameter, which is the minimum distance between the robot 
        # and a grid zone for it to be considered in contact with the zone
        self.contact_threshold = rospy.get_param("~contact_threshold", 1.0)

        # Load the no_fill_timeout parameter, which is the time in seconds after which, if no
        # detection is at the grid zone and the grid zone is in view of the camera, it will be marked as empty
        self.no_fill_timeout = rospy.get_param("~no_fill_timeout", 1.0)

        # Load the linear_velocity_threshold parameter, which is the maximum linear absolute velocity of the
        # robot at which the grid zone state can be updated
        self.linear_velocity_threshold = rospy.get_param("~linear_velocity_threshold", 1.0)

        # Load the angular_velocity_threshold parameter, which is the maximum angular absolute velocity of the
        # robot at which the grid zone state can be updated
        self.angular_velocity_threshold = rospy.get_param("~angular_velocity_threshold", 1.0)

        # Load the global_frame_id parameter, which is the name of the global frame (typically "map")
        self.map_link = rospy.get_param("~global_frame_id", "map")

        # Load the base_frame_id parameter, which is the name of the robot's base frame (typically "base_link")
        self.base_link = rospy.get_param("~base_frame_id", "base_link")

        # Load the nt_host parameter, which is the IP address of the NetworkTables server
        self.nt_host = rospy.get_param("~nt_host", "10.0.88.2")


        # Initialize NetworkTables with the specified IP address of the server
        NetworkTables.initialize(server=self.nt_host)

        # Get the "SmartDashboard" table from NetworkTables
        self.smart_dashboard_table = NetworkTables.getTable("SmartDashboard")

        # Get the "grid_tracking" subtable from the "SmartDashboard" table
        self.grid_tracking_table = self.smart_dashboard_table.getSubTable("grid_tracking")

        # Get the "column_state" subtable from the "grid_tracking" table
        self.grid_tracking_column_state = self.grid_tracking_table.getSubTable("column_state")


        # Initialize the "status" entry in the "grid_tracking" table to store the current tracking status
        self.status_entry = self.grid_tracking_table.getEntry("status")

        # Initialize the "nearest_zone_distance" entry in the "grid_tracking" table to store the distance to the nearest zone
        self.nearest_zone_entry = self.grid_tracking_table.getEntry("nearest_zone_distance")

        # Initialize the "nearest_zone_serial" entry in the "grid_tracking" table to store the serial number of the nearest zone
        self.nearest_zone_serial_entry = self.grid_tracking_table.getEntry("nearest_zone_serial")

        # Initialize the "nearest_zone_row_index" entry in the "grid_tracking" table to store the row index of the nearest zone
        self.nearest_zone_row_index = self.grid_tracking_table.getEntry("nearest_zone_row_index")


        # Load the grid zones from the CSV file specified by grid_zones_path
        self.grid_zones = GridZoneManager.from_file(grid_zones_path)

        # Initialize the camera model to None, to be set later
        self.camera_model: Optional[PinholeCameraModel] = None

        # Create a buffer to store the tf2 (transform) data
        self.tf_buffer = tf2_ros.Buffer()

        # Create a TransformListener to listen for transforms
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Create a Lock object to ensure thread-safe access to the robot's transform data
        self.robot_tf_lock = Lock()

        # Create a Lock object to ensure thread-safe access to the grid zone data
        self.zones_lock = Lock()

        # Create a TransformStamped object to store the robot's global transform (map to base_link)
        self.global_tf = TransformStamped()
        self.global_tf.header.frame_id = self.map_link
        self.global_tf.child_frame_id = self.base_link
        self.global_tf.transform.rotation.w = 1.0

        # Initialize the robot's velocity data to store linear and angular velocities
        self.robot_velocity = Velocity()


        # Create a Publisher to publish MarkerArray messages for visualization of grid tracking in Rviz
        self.markers_pub = rospy.Publisher(
            "grid_tracking/markers", MarkerArray, queue_size=10
        )

        # Create a Subscriber to listen for Detection3DArray messages containing information about
        # the currently visible game objects
        self.detections_sub = rospy.Subscriber(
            "detections", Detection3DArray, self.detections_callback, queue_size=10
        )

        # Create a Subscriber to listen for Odometry messages containing the robot's current position,
        # orientation, and velocity (both linear and angular)
        self.odom_sub = rospy.Subscriber(
            "odom", Odometry, self.odom_callback, queue_size=10
        )

        # Create a Subscriber to listen for CameraInfo messages containing the camera's intrinsic parameters
        self.camera_info_sub = rospy.Subscriber("camera_info", CameraInfo, self.info_callback, queue_size=1)

        # Log that the grid tracker has been initialized
        rospy.loginfo("Grid tracker initialized")

    def update_global_tf(self) -> None:
        """
        Update the stored global transform (map to base_link) of the robot using the latest
        available transform data from the tf_buffer.
        """
        # Look up the current transform between the global frame (map) and the robot's base frame (base_link)
        global_tf = lookup_transform(self.tf_buffer, self.map_link, self.base_link)

        # If the transform is found, update the stored global transform with the new data
        if global_tf is not None:
            with self.robot_tf_lock:
                self.global_tf = global_tf

    def info_callback(self, msg: CameraInfo) -> None:
        """
        Callback function for the "camera_info" subscriber. It initializes the camera model
        using the received CameraInfo message and then unregisters the subscriber.

        Args:
            msg (CameraInfo): The received CameraInfo message containing the camera's intrinsic parameters.
        """
        # Log that the camera model has been received
        rospy.loginfo("Got camera model")

        # Unregister the "camera_info" subscriber since the camera model only needs to be set once
        self.camera_info_sub.unregister()

        # Initialize the PinholeCameraModel using the received CameraInfo message
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(msg)

    def odom_callback(self, msg: Odometry) -> None:
        """
        Callback function for the "odom" subscriber. It updates the robot's velocity data
        using the received Odometry message.

        Args:
            msg (Odometry): The received Odometry message containing the robot's current position,
                            orientation, and velocity.
        """
        # Update the robot's velocity using the linear and angular velocities from the received Odometry message
        self.robot_velocity = Velocity.from_ros_twist(msg.twist.twist)

    def detections_callback(self, msg: Detection3DArray) -> None:
        """
        Callback function for the "detections" subscriber. Processes the received Detection3DArray
        message to update the grid zone states, utilizing helper functions to manage the process.

        Args:
            msg (Detection3DArray): The received Detection3DArray message containing information about
                                    the currently visible game objects in the grid zones.
        """
        # Return without updating the grid zones if the robot's linear or angular velocity is above the respective thresholds
        if self.should_skip_due_to_velocity():
            return

        # Get the global transform
        global_tf = self.get_global_transform(msg)
        if global_tf is None:
            return

        # Get the reverse transform if the camera model is available
        reverse_tf = self.get_reverse_transform()

        with self.zones_lock:
            # Update the grid zones' in_view and filled properties
            if reverse_tf is not None:
                self.update_grid_zones(reverse_tf)

            # Process each detection and update the nearest grid zone's filled property
            self.process_detections(msg, global_tf)

    def should_skip_due_to_velocity(self) -> bool:
        """
        Determines whether to skip grid zone updates based on the robot's linear and angular velocities.

        Returns:
            bool: True if grid zone updates should be skipped, False otherwise.
        """
        return (
            self.robot_velocity.magnitude() > self.linear_velocity_threshold
            or abs(self.robot_velocity.theta) > self.angular_velocity_threshold
        )

    def get_global_transform(self, msg: Detection3DArray) -> Optional[TransformStamped]:
        """
        Get the global transform based on the header frame_id of the received Detection3DArray message.

        Args:
            msg (Detection3DArray): The received Detection3DArray message.

        Returns:
            Optional[TransformStamped]: The global transform if available, None otherwise.
        """
        if msg.header.frame_id == self.base_link:
            self.update_global_tf()
            return self.global_tf
        else:
            return lookup_transform(self.tf_buffer, self.map_link, msg.header.frame_id)

    def get_reverse_transform(self) -> Optional[TransformStamped]:
        """
        Get the reverse transform from the camera model's frame to the map frame if the camera model is available.

        Returns:
            Optional[TransformStamped]: The reverse transform if available, None otherwise.
        """
        if self.camera_model is not None:
            return lookup_transform(self.tf_buffer, self.camera_model.tfFrame(), self.map_link)
        else:
            return None

    def update_grid_zones(self, reverse_tf: TransformStamped) -> None:
        """
        Update the grid zones' in_view and filled properties based on the reverse transform.
        The reverse transform is used to put global positions relative to the camera, which helps
        in checking if the grid zones are in view of the camera or not.

        Args:
            reverse_tf (TransformStamped): The reverse transform.
        """
        # Iterate through all the grid zones
        for zone in self.grid_zones.zones:
            # Update the in_view property of the zone based on whether it's in view of the camera
            zone.set_in_view(self.is_zone_in_view(reverse_tf, zone))

            # Get the current time
            now = time.monotonic()

            # Check if the zone has been filled for more than the no_fill_timeout duration
            did_fill_timeout = now - zone.filled_time > self.no_fill_timeout

            # Check if the zone has been out of view for more than the no_fill_timeout duration
            did_not_in_view_timeout = now - zone.not_in_view_time > self.no_fill_timeout

            # If the zone is in view and both timeouts have passed, set the zone's filled property to False
            if zone.in_view and did_fill_timeout and did_not_in_view_timeout:
                zone.filled = False

    def process_detections(self, msg: Detection3DArray, global_tf: TransformStamped) -> None:
        """
        Process each detection in the received Detection3DArray message and update the nearest grid zone's filled property.
        This function iterates through all the detections, transforms their poses to global coordinates, and then finds the
        nearest grid zone. If the distance to the nearest grid zone is less than the contact threshold, the grid zone is
        considered filled with a game object.

        Args:
            msg (Detection3DArray): The received Detection3DArray message containing information about the game objects.
            global_tf (TransformStamped): The global transform used to transform detection poses from their local
                                          coordinates to global coordinates.
        """
        
        # Ensure the detections list in the message is not None
        assert msg.detections is not None

        # Iterate through all the detections in the message
        for detection in msg.detections:
            # Create a PoseStamped object for the detection's pose
            detection_pose = PoseStamped()
            detection_pose.header.frame_id = msg.header.frame_id
            detection_pose.pose = detection.bbox.center

            # Transform the detection pose from local to global coordinates
            global_pose = tf2_geometry_msgs.do_transform_pose(detection_pose, global_tf)

            # Find the nearest grid zone and its distance from the transformed detection pose
            nearest_zone, distance = self.grid_zones.get_nearest(
                global_pose.pose.position.x,
                global_pose.pose.position.y,
                global_pose.pose.position.z
            )

            # Set the filled property of the nearest grid zone based on the distance
            nearest_zone.set_filled(distance < self.contact_threshold)

    def is_zone_in_view(self, reverse_tf: TransformStamped, zone: GridZone) -> bool:
        """
        Check if the given zone is within the camera's field of view.

        Args:
            reverse_tf (TransformStamped): The transform used to convert grid zone positions from global
                                            coordinates to camera-relative coordinates.
            zone (GridZone): The grid zone to check.

        Returns:
            bool: True if the zone is within the camera's field of view, False otherwise.
        """
        # If the reverse transform or camera model is not available, return False
        if reverse_tf is None or self.camera_model is None:
            return False

        # Create a PoseStamped object for the grid zone's position
        grid_pose = PoseStamped()
        grid_pose.header.frame_id = self.map_link
        grid_pose.pose.position.x = zone.x
        grid_pose.pose.position.y = zone.y
        grid_pose.pose.position.z = zone.z

        # Transform the grid zone's position from global coordinates to camera-relative coordinates
        camera_relative_grid = tf2_geometry_msgs.do_transform_pose(grid_pose, reverse_tf)

        # If the transformed position's z-coordinate is negative, it's behind the camera, so return False
        if camera_relative_grid.pose.position.z < 0.0:
            return False

        # Project the camera-relative position onto the camera's image plane
        px = self.camera_model.project3dToPixel((
            camera_relative_grid.pose.position.x,
            camera_relative_grid.pose.position.y,
            camera_relative_grid.pose.position.z,
        ))

        # Check if the projected pixel coordinates are within the camera's image bounds
        return 0 < px[0] < self.camera_model.width and 0 < px[1] < self.camera_model.height  # type: ignore

    def get_robot_pose(self) -> Optional[Pose2d]:
        """
        Get the robot's current 2D pose in the global frame.

        Returns:
            Optional[Pose2d]: The robot's current 2D pose in the global frame, or None if the transformation fails.
        """
        # Create a PoseStamped object representing the robot's base in its local frame
        zero_pose_base = PoseStamped()
        zero_pose_base.header.frame_id = self.base_link
        zero_pose_base.pose.orientation.w = 1.0

        # Transform the robot's local pose to the global frame using the stored global transform
        global_pose = tf2_geometry_msgs.do_transform_pose(zero_pose_base, self.global_tf)

        # Convert the transformed global pose to a 2D pose and return it
        return Pose2d.from_ros_pose(global_pose.pose)

    def report_state(self) -> None:
        """
        Report the current state of the nearest grid column to the NetworkTables.
        """
        # Get the robot's current 2D pose in the global frame
        robot_pose = self.get_robot_pose()
        
        # If the robot pose is invalid, update the status entry and return
        if robot_pose is None:
            self.status_entry.setString("Invalid robot pose")
            return

        # Acquire the zones lock to safely access the grid zones
        with self.zones_lock:
            # Find the nearest grid zone and its distance from the robot
            nearest_zone, distance = self.grid_zones.get_nearest(robot_pose.x, robot_pose.y, 0.0)
            
            # Update the NetworkTables entries with the distance and zone serial information
            self.nearest_zone_entry.setDouble(distance)
            self.nearest_zone_serial_entry.setString(self.get_serial(nearest_zone))

            # Get the row containing the nearest grid zone
            row = self.grid_zones.get_row(nearest_zone.alliance, nearest_zone.row)
            
            # Initialize a status dictionary for the row's column types
            status = {ColumnType.HIGH: False, ColumnType.MID: False, ColumnType.LOW: False}
            row_index = nearest_zone.row
            
            # Update the status dictionary with the filled status of each zone in the row
            for zone in row:
                status[zone.column] = zone.filled
            
            # Update the NetworkTables entries for each column type with their filled status
            for column_type, filled in status.items():
                self.grid_tracking_column_state.getEntry(column_type.value).setBoolean(filled)
            
            # Update the NetworkTables entry for the row index
            self.nearest_zone_row_index.setDouble(row_index)
        
        # Update the status entry with the "OK" status
        self.status_entry.setString("OK")

    def publish_grid_state_markers(self) -> None:
        """
        Publish the current state of the grid zones as visualization markers.
        """
        # Initialize a MarkerArray to store the grid state markers
        marker_array = MarkerArray()
        assert marker_array.markers is not None

        # Acquire the zones lock to safely access the grid zones
        with self.zones_lock:
            # Iterate through the grid zones and convert each zone to its corresponding visualization markers
            for zone in self.grid_zones.zones:
                markers = self.grid_zone_to_marker(zone)
                
                # Add the generated markers to the MarkerArray
                marker_array.markers.extend(markers)

        # Publish the MarkerArray containing the grid state markers
        self.markers_pub.publish(marker_array)

    def grid_zone_to_marker(self, grid_zone: GridZone) -> List[Marker]:
        """
        Convert a GridZone object into a list of visualization markers (sphere and text).

        Args:
            grid_zone (GridZone): The GridZone object to convert into visualization markers.

        Returns:
            List[Marker]: A list containing the generated sphere and text markers for the given grid zone.
        """
        # Create a PoseStamped for the grid zone's position and orientation
        pose = PoseStamped()
        pose.header.frame_id = self.map_link
        pose.pose.position.x = grid_zone.x
        pose.pose.position.y = grid_zone.y
        pose.pose.position.z = grid_zone.z
        pose.pose.orientation.w = 1.0

        # Determine the color based on the grid zone's state (filled, in_view, alliance)
        if grid_zone.filled:
            color = (0.0, 0.0, 0.0, 0.95)
        else:
            alpha = 0.95 if grid_zone.in_view else 0.25
            if grid_zone.alliance == Alliance.BLUE:
                color = (0.0, 0.0, 1.0, alpha)
            else:
                color = (1.0, 0.0, 0.0, alpha)

        # Create the sphere marker for the grid zone
        sphere_marker = self.make_marker(self.get_serial(grid_zone), pose, color)
        sphere_marker.ns = "sphere" + sphere_marker.ns
        sphere_marker.type = Marker.SPHERE

        # Create the text marker for the grid zone
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

        # Return the list of generated markers (sphere and text)
        return [sphere_marker, text_marker]

    def make_marker(self, name: str, pose: PoseStamped, color: Tuple[float, float, float, float]) -> Marker:
        """
        Create a basic visualization marker with the given name, pose, and color.

        Args:
            name (str): The unique name of the marker.
            pose (PoseStamped): The pose (position and orientation) of the marker.
            color (Tuple[float, float, float, float]): A tuple representing the RGBA color of the marker.

        Returns:
            Marker: The created marker with the given properties.
        """
        # Initialize a new Marker object
        marker = Marker()
        marker.action = Marker.ADD
        marker.pose = copy.deepcopy(pose.pose)
        marker.header = pose.header
        marker.lifetime = rospy.Duration(1.0)  # type: ignore
        marker.ns = name
        marker.id = 0  # all waypoint names should be unique

        # Set the scale of the marker
        scale_vector = Vector3()
        scale_vector.x = self.contact_threshold * 2
        scale_vector.y = self.contact_threshold * 2
        scale_vector.z = self.contact_threshold * 2
        marker.scale = scale_vector

        # Set the color of the marker
        marker.color = ColorRGBA(
            r=color[0],
            g=color[1],
            b=color[2],
            a=color[3],
        )

        return marker

    @staticmethod
    def get_serial(zone: GridZone) -> str:
        """
        Generate a unique serial string for a given grid zone.

        Args:
            zone (GridZone): The grid zone for which to generate the serial string.

        Returns:
            str: The unique serial string for the given grid zone.
        """
        return f"{zone.alliance.value}-{zone.column.value}-{zone.row}"
    
    def run(self):
        """
        The main loop of the GridTracker node. It continuously updates the global transform,
        reports the state, publishes grid state markers, and sleeps at a specified rate.
        """
        # Set the loop rate to 10 Hz
        rate = rospy.Rate(10)
        
        # Keep running the loop until the ROS node is shut down
        while not rospy.is_shutdown():
            # Update the global transform
            self.update_global_tf()
            # Report the current state
            self.report_state()
            # Publish grid state markers for visualization
            self.publish_grid_state_markers()
            # Sleep for the specified rate
            rate.sleep()


if __name__ == '__main__':
    node = GridTracker()
    node.run()
