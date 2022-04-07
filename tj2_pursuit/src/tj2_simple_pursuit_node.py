#!/usr/bin/env python3

import math

import rospy
import actionlib

import tf2_ros
import tf2_geometry_msgs

from nav_msgs.msg import Odometry

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist

from std_msgs.msg import Float64

from vision_msgs.msg import Detection3DArray

from tj2_pursuit.msg import PursueObjectAction, PursueObjectGoal, PursueObjectResult

from tj2_tools.particle_filter.state import FilterState, SimpleFilter, DeltaTimer
from tj2_tools.yolo.utils import get_label, read_class_names
from tj2_tools.transforms import lookup_transform
# from tj2_tools.motion_profile import TrapezoidalProfileLive
# from tj2_tools.motion_profile import TrapezoidalProfile
from tj2_tools.motion_profile import PIDController
from tj2_tools.predictions.predictor import BouncePredictor


class Tj2SimplePursuit:
    def __init__(self):
        self.name = "tj2_simple_pursuit"
        rospy.init_node(
            self.name
        )

        self.class_names_path = rospy.get_param("~class_names_path", "objects.names")
        self.class_names = read_class_names(self.class_names_path)

        self.rotate_kP = rospy.get_param("~rotate_kP", 1.0)
        self.rotate_kI = rospy.get_param("~rotate_kI", 0.0)
        self.rotate_kD = rospy.get_param("~rotate_kD", 0.0)
        
        self.max_angular_vel = rospy.get_param("~max_angular_vel", 5.0)
        self.max_linear_vel = rospy.get_param("~max_linear_vel", 2.0)
        self.max_linear_accel = rospy.get_param("~max_linear_accel", 1.0)

        self.linear_kP = rospy.get_param("~linear_kP", 1.0)
        self.linear_kI = rospy.get_param("~linear_kI", 0.0)
        self.linear_kD = rospy.get_param("~linear_kD", 0.0)

        self.object_kF = rospy.get_param("~object_kF", 0.0)

        self.object_reached_threshold = rospy.get_param("~object_reached_threshold", 0.1)
        self.command_rate = rospy.get_param("~command_rate", 15.0)
        self.no_object_timeout = rospy.Duration(rospy.get_param("~no_object_timeout", 2.0))

        self.enable_linear_vel = rospy.get_param("~enable_linear_vel", True)
        self.enable_prediction = rospy.get_param("~enable_prediction", False)

        self.camera_base_frame = rospy.get_param("~camera_base_frame", "base_link")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_link")

        self.distance_filter = SimpleFilter(0.9)
        self.normalize_error = 1.0
        error = self.get_angle_error(math.pi)
        if error > 0.0:
            self.normalize_error = 1.0 / error
        else:
            raise ValueError("Angle error function returned an value: %0.4f" % error)
        
        self.delta_timer = DeltaTimer()
        self.detection_loop_timer = DeltaTimer()

        self.angular_pid = PIDController(
            kp=self.rotate_kP,
            ki=self.rotate_kI,
            kd=self.rotate_kD,
        )
        self.linear_pid = PIDController(
            kp=self.linear_kP,
            ki=self.linear_kI,
            kd=self.linear_kD,
        )
        self.limited_linear_command = 0.0
        # self.linear_vel_controller = TrapezoidalProfileLive(
        #     dict(
        #         kp=self.linear_kP,
        #         ki=self.linear_kI,
        #         kd=self.linear_kD,
        #     ),
        #     dict(
        #         max_speed=self.max_linear_vel,
        #         max_accel=self.max_linear_accel
        #     )
        # )

        self.predictor = BouncePredictor(  # TODO: make dynamically configurable
            v_max_robot=4.0,
            past_window_size=4,
            vx_std_dev_threshold=1.0,
            vy_std_dev_threshold=1.0
        )

        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.camera_tilt_pub = rospy.Publisher("joint_command/camera_joint", Float64, queue_size=10)
        self.follow_object_goal_pub = rospy.Publisher("follow_object_goal", PoseStamped, queue_size=10)

        self.tracking_object_name = ""
        self.tracking_state = None
        self.tracking_tilt_state = None
        self.object_is_in_view = False
        self.no_object_timer = rospy.Time.now()

        self.odom_state = FilterState()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.detections_sub = rospy.Subscriber("detections", Detection3DArray, self.obj_callback, queue_size=15)
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback, queue_size=15)

        self.pursue_object_server = actionlib.SimpleActionServer("pursue_object", PursueObjectAction, self.pursue_object_callback, auto_start=False)
        self.pursue_object_server.start()

        rospy.loginfo("%s is ready" % self.name)

    def odom_callback(self, msg):
        self.odom_state = FilterState.from_odom(msg)

    def pursue_object_callback(self, goal):
        rate = rospy.Rate(self.command_rate)
        start_timer = rospy.Time.now()
        detection_timeout = goal.timeout
        rospy.loginfo("Pursuit received goal: %s" % str(goal))
        self.tracking_object_name = goal.object_name
        self.prev_pose_stamped = None
        success = False
        self.tracking_state = None
        self.tracking_tilt_state = None
        self.object_is_in_view = False
        self.no_object_timer = rospy.Time.now()
        self.limited_linear_command = 0.0

        while True:
            rate.sleep()
            if detection_timeout > rospy.Duration(0.0) and rospy.Time.now() - start_timer > detection_timeout:
                self.pursue_object_server.set_aborted(PursueObjectResult(success))
                rospy.logwarn("Timed out while searching for '%s'" % self.tracking_object_name)
                break
            if rospy.is_shutdown() or self.pursue_object_server.is_preempt_requested():
                self.pursue_object_server.set_preempted(PursueObjectResult(success))
                rospy.logwarn("Preempted while searching for '%s'" % self.tracking_object_name)
                break

            if self.pursue_object():
                success = True
                self.pursue_object_server.set_succeeded(PursueObjectResult(success))
                rospy.loginfo("Successfully reached '%s'" % self.tracking_object_name)
                break

        self.cancel_goal()

    def obj_callback(self, detections_msg):
        tracking_state, tracking_tilt_state = self.get_nearest_detection(self.tracking_object_name, detections_msg)

        if len(self.tracking_object_name) > 0 and (tracking_state is None or tracking_tilt_state is None):
            self.object_is_in_view = False
            rospy.logwarn("No object is available to pursue!")
        else:
            self.no_object_timer = rospy.Time.now()
            self.object_is_in_view = True
            if tracking_state is None:
                self.tracking_state = None
            else:
                if self.enable_prediction:
                    self.tracking_state = self.predictor.get_robot_intersection(tracking_state, self.odom_state)
                else:
                    self.tracking_state = tracking_state
            self.tracking_tilt_state = tracking_tilt_state

    def get_nearest_detection(self, object_name, detections_msg):
        nearest_pose = None
        nearest_dist = None
        for detection in detections_msg.detections:
            if len(object_name) > 0:
                label, index = get_label(self.class_names, detection.results[0].id)
                if label != object_name:
                    continue
            detection_pose = detection.results[0].pose.pose
            detection_dist = self.get_distance(detection_pose)
            if nearest_dist is None or detection_dist < nearest_dist:
                nearest_pose = PoseStamped()
                nearest_pose.pose = detection_pose
                nearest_pose.header = detection.header
                nearest_dist = detection_dist
        
        target_state = None
        if nearest_pose is not None:
            tracking_state_base = FilterState.from_ros_pose(nearest_pose.pose)
            if self.object_kF > 0.0:
                dt = self.detection_loop_timer.update(rospy.Time.now())
                if dt > 0.0:
                    motion_compensate = FilterState(theta=self.object_kF * self.odom_state.vt * dt)
                    tracking_state_base = tracking_state_base.relative_to(motion_compensate)
            odom_state = self.get_odom_state()
            if odom_state is not None:
                target_state = tracking_state_base.relative_to(odom_state)
                target_state.stamp = detections_msg.header.stamp.to_sec()
            
        # target_pose = self.get_pose_in_odom(nearest_pose)
        target_tilt_pose = self.get_pose_in_camera_tilt(nearest_pose)  # this could create an issue if the camera isn't well centered on the intake
        if target_state is not None and target_tilt_pose is not None:
            target_pose = PoseStamped()
            target_pose.header.frame_id = self.odom_frame
            target_pose.pose = target_state.to_ros_pose()
            self.follow_object_goal_pub.publish(target_pose)

            # nearest_state = FilterState.from_ros_pose(target_pose.pose)
            target_tilt_state = FilterState.from_ros_pose(target_tilt_pose.pose)
            target_tilt_state.stamp = target_tilt_pose.header.stamp.to_sec()
            return target_state, target_tilt_state
        else:
            return None, None
    
    def get_distance(self, pose1, pose2=None):
        if pose2 is None:
            x = pose1.position.x
            y = pose1.position.y
        else:
            x1 = pose1.position.x
            y1 = pose1.position.y
            x2 = pose2.position.x
            y2 = pose2.position.y
            x = x2 - x1
            y = y2 - y1
        return math.sqrt(x * x + y * y)

    def get_pose_in_camera_tilt(self, pose_stamped):
        if pose_stamped is None:
            return pose_stamped
        frame = pose_stamped.header.frame_id
        camera_tf = lookup_transform(self.tf_buffer, self.camera_base_frame, frame)
        if camera_tf is None:
            return None
        camera_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, camera_tf)
        return camera_pose

    def get_pose_in_odom(self, pose_stamped):
        if pose_stamped is None:
            return pose_stamped
        frame = pose_stamped.header.frame_id
        odom_tf = lookup_transform(self.tf_buffer, self.odom_frame, frame)
        if odom_tf is None:
            return None
        odom_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped, odom_tf)
        return odom_pose

    def get_odom_state(self):
        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.pose.orientation.w = 1.0
        odom_pose = self.get_pose_in_odom(pose)
        if odom_pose is None:
            rospy.logwarn_throttle(1.0, "Failed to get odom transform!")
            return None
        else:
            return FilterState.from_ros_pose(odom_pose.pose)

    def cancel_goal(self):
        self.tracking_object_name = ""
        self.object_is_in_view = False
        self.stop_motors()

    def stop_motors(self):
        # stop chassis from moving
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0

        # reset tilt position to default location
        tilt = Float64()
        tilt.data = 0.0
        self.camera_tilt_pub.publish(tilt)

        for _ in range(10):
            self.cmd_vel_pub.publish(stop_msg)

    def get_angle_error(self, angle):
        return math.atan(angle * 2.0) * self.normalize_error

    def pursue_object(self):
        if self.tracking_state is None or self.tracking_tilt_state is None:
            return False
        odom_state = self.get_odom_state()
        if odom_state is None:
            return False
        track_robot_relative = self.tracking_state.relative_to_reverse(odom_state)
        distance = self.distance_filter.update(
            track_robot_relative.distance(states="xy")
        )
        rospy.loginfo("Tracking: d=%0.4f, X=%0.4f, Y=%0.4f" % (distance, track_robot_relative.x, track_robot_relative.y))
        if distance < self.object_reached_threshold:
            rospy.loginfo("Arrived at object: %0.4f" % distance)
            return True

        # if cargo is to the left, turn left
        # if cargo is above, tilt camera up
        # drive forward to the tracked object following a trapezoidal profile
        
        now = rospy.Time.now()
        dt = self.delta_timer.dt(now.to_sec())
        if dt == 0.0:
            return False
        error = self.get_angle_error(track_robot_relative.heading())
        # error = track_robot_relative.heading()
        # angular_velocity = self.rotate_kP * error
        angular_velocity = self.angular_pid.update(0.0, -error, dt)
        if abs(angular_velocity) > self.max_angular_vel:
            angular_velocity = math.copysign(self.max_angular_vel, angular_velocity)

        # if now - self.no_object_timer > self.no_object_timeout:
        #     distance = 0.0
        # self.linear_vel_controller.reset_position(distance)
        if rospy.Time.now() - self.no_object_timer < self.no_object_timeout:
            target_dist = distance
        else:
            target_dist = 0.0
        # self.linear_vel_controller.set_target_velocity(0.0)
        # self.linear_vel_controller.set_target_position(target_dist)
        
        twist = Twist()
        if self.enable_linear_vel:
            # twist.linear.x = self.linear_vel_controller.calculate_velocity(distance, self.odom_state.vx, dt)
            # twist.linear.x = self.linear_vel_controller.calculate_command_velocity(odom_state.vx, dt)
            linear_command = self.linear_pid.update(0.0, -target_dist, dt)
            self.limited_linear_command += math.copysign(self.max_linear_accel * dt, linear_command - self.limited_linear_command)
            self.limited_linear_command = min(self.max_linear_vel, max(-self.max_linear_vel, self.limited_linear_command))
            twist.linear.x = self.limited_linear_command
        twist.angular.z = angular_velocity
        self.cmd_vel_pub.publish(twist)

        tilt_position = self.tracking_tilt_state.heading(states="xz")
        tilt = Float64()
        tilt.data = tilt_position
        self.camera_tilt_pub.publish(tilt)

        return False

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    node = Tj2SimplePursuit()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting %s node" % node.name)

