import math
import rospy
import actionlib
import geometry_msgs

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from smach import State, StateMachine

class GoToWaypointState(State):
    def __init__(self):
        super(GoToWaypointState, self).__init__(
            outcomes=["success", "preempted", "failure", "finished"],
            input_keys=["waypoints", "waypoint_index_in", "action_server"],
            output_keys=["waypoints", "waypoint_index_out", "action_server"]
        )
    
        self.intermediate_tolerance = rospy.get_param("~intermediate_tolerance", 0.0)
        self.action_result = "success"
        self.goal_pose_stamped = None
        self.current_waypoint_index = 0
        self.num_waypoints = 0

    def execute(self, userdata):
        self.action_result = "success"
        self.action_server = userdata.action_server

        self.num_waypoints = len(userdata.waypoints)
        self.current_waypoint_index = userdata.waypoint_index_in

        if userdata.waypoint_index_in >= self.num_waypoints:
            self.action_server.set_succeeded(True)
            return "finished"
        
        if self.action_server.is_continuous:
            return self.execute_continuous(userdata)
        else:
            return self.execute_discontinuous(userdata)
    
    def execute_continuous(self, userdata):
        waypoints = userdata.waypoints

        goal = MoveBaseGoal()
        goal.target_poses.header.frame_id = waypoints[0].header.frame_id
        for waypoint in waypoints:
            if type(waypoint) != geometry_msgs.PoseStamped:
                rospy.logerr("Waypoint isn't of type PoseStampted! %s" % repr(waypoint))
                return "failure"
            if waypoint.header.frame_id != goal.target_poses.header.frame_id:
                rospy.logerr("Waypoints have inconsistent frame_ids! %s != %s" % (waypoint.header.frame_id, goal.target_poses.header.frame_id))
                return "failure"

            goal.target_poses.poses.append(waypoint.pose)
        
        self.action_server.move_base.send_goal(goal, feedback_cb=self.move_base_feedback, done_cb=self.move_base_done)
        self.action_server.move_base.wait_for_result()

        if self.action_result != "success":
            return self.action_result

        move_base_result = self.action_server.move_base.get_result()
        if bool(move_base_result):
            userdata.waypoint_index_out = len(waypoints)
            return "success"
        else:
            return "failure"

    def execute_discontinuous(self, userdata):
        waypoint_pose = userdata.waypoints[userdata.waypoint_index_in]
        self.goal_pose_stamped = waypoint_pose

        goal = MoveBaseGoal()
        goal.target_poses.header.frame_id = waypoint_pose.header.frame_id
        goal.target_poses.poses.append(waypoint_pose.pose)
        
        rospy.loginfo("Going to position (%s, %s)" % (waypoint_pose.pose.position.x, waypoint_pose.pose.position.y))

        self.action_server.move_base.send_goal(goal, done_cb=self.move_base_done)
        self.action_server.move_base.wait_for_result()

        if self.action_result != "success":
            return self.action_result

        move_base_result = self.action_server.move_base.get_result()
        if bool(move_base_result):
            userdata.waypoint_index_out = userdata.waypoint_index_in + 1
            return "success"
        else:
            return "failure"
    
    def move_base_feedback(self, feedback):
        if rospy.is_shutdown():
            rospy.loginfo("Received abort. Cancelling waypoint goal")
            self.action_server.set_aborted()
            self.action_result = "failure"
            self.action_server.move_base.cancel_goal()

        if self.action_server.is_preempt_requested():
            rospy.loginfo("Received preempt. Cancelling waypoint goal")
            self.action_server.set_preempted()
            self.action_result = "preempted"
            self.action_server.move_base.cancel_goal()
        
        # rospy.loginfo("feedback: %s" % str(feedback))
        if self.intermediate_tolerance != 0.0 and self.current_waypoint_index < self.num_waypoints - 1:
            dist = self.get_xy_dist(feedback.base_position, self.goal_pose_stamped)
            if dist <= self.intermediate_tolerance:
                rospy.loginfo("Robot is close enough to goal. Moving on")
                self.action_server.move_base.cancel_goal()
                self.action_result = "success"

    
    def move_base_done(self, goal_status, result):
        rospy.loginfo("move_base finished")
    
    def get_xy_dist(self, pose1, pose2):
        # pose1 and pose2 are PoseStamped
        x1 = pose1.pose.position.x
        y1 = pose1.pose.position.y
        x2 = pose2.pose.position.x
        y2 = pose2.pose.position.y
        
        dx = x2 - x1
        dy = y2 - y1

        return math.sqrt(dx * dx + dy * dy)


class WaypointStateMachine(object):
    def __init__(self):
        self.sm = StateMachine(outcomes=["success", "failure", "preempted"])
        self.outcome = None
        self.action_server = None
        self.is_continuous = False

        self.move_base_namespace = rospy.get_param("~move_base_namespace", "/move_base")
        self.move_base = actionlib.SimpleActionClient(self.move_base_namespace, MoveBaseAction)
        rospy.loginfo("Connecting to move_base...")
        self.move_base.wait_for_server()
        rospy.loginfo("move_base connected")

        with self.sm:
            StateMachine.add(
                "GOTO_WAYPOINT", GoToWaypointState(),
                transitions={
                    "success": "GOTO_WAYPOINT",
                    "finished": "success",
                    "failure": "failure",
                    "preempted": "preempted"
                },
                remapping={
                    "waypoints": "sm_waypoints",
                    "is_continuous": "sm_continuous",
                    "waypoint_index_in": "sm_waypoint_index",
                    "waypoint_index_out": "sm_waypoint_index",
                    "action_server": "sm_action_server",
                }
            )

    def execute(self, is_continuous, waypoints, action_server):
        rospy.loginfo("To cancel the waypoint follower, run: 'rostopic pub -1 /tj2/follow_path/cancel actionlib_msgs/GoalID -- {}'")
        rospy.loginfo("To cancel the current goal, run: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")

        self.is_continuous = is_continuous

        self.sm.userdata.sm_waypoint_index = 0
        self.sm.userdata.sm_waypoints = waypoints
        self.sm.userdata.sm_action_server = action_server
        self.outcome = self.sm.execute()
        return self.outcome
