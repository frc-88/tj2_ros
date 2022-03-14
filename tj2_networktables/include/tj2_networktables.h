#pragma once

#include <math.h>

#include <boost/array.hpp>
#include <vector>
#include <boost/range/algorithm.hpp>
#include <iostream>
#include <cassert>

#include "ntcore.h"

#include "ros/ros.h"
#include "ros/console.h"

#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "actionlib/client/simple_client_goal_state.h"

#include "tf/transform_listener.h"

#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include "tj2_waypoints/FollowPathGoal.h"
#include "tj2_waypoints/FollowPathAction.h"

#include "tj2_waypoints/Waypoint.h"
#include "tj2_waypoints/WaypointArray.h"

#include "tj2_networktables/OdomReset.h"

#include "networktables/EntryListenerFlags.h"

using namespace std;

// NT callbacks
void ping_callback(void* data, const NT_EntryNotification* event);


typedef enum {
    IDLE=0,
    RUNNING=1,
    FINISHED=2,
    FAILED=3,
    INVALID=-1,
} GoalStatus;


int sign_of(double x) {
    return (x > 0) - (x < 0);
}

template<size_t Size, class Container>
boost::array<typename Container::value_type, Size> as_array(const Container &cont)
{
    assert(cont.size() == Size);
    boost::array<typename Container::value_type, Size> result;
    boost::range::copy(cont, result.begin());
    return result;
}

class TJ2NetworkTables {
private:
    ros::NodeHandle nh;  // ROS node handle

    // Parameters
    int _nt_port;
    double _update_interval;

    bool _publish_odom_tf;
    string _base_frame;
    string _odom_frame;
    string _map_frame;
    string _imu_frame;

    double _cmd_vel_timeout_param;
    double _min_linear_cmd;
    double _min_angular_z_cmd;
    double _zero_epsilon;

    double _pose_estimate_x_std, _pose_estimate_y_std, _pose_estimate_theta_std_deg;
    string _pose_estimate_frame_id;

    std::vector<std::string> _joint_names;

    std::vector<double> _odom_covariance;
    std::vector<double> _twist_covariance;
    std::vector<double> _imu_orientation_covariance;
    std::vector<double> _imu_angular_velocity_covariance;
    std::vector<double> _imu_linear_acceleration_covariance;

    // NT entries
    NT_Inst _nt;
    string _base_key;

    // odom entries
    NT_Entry _odom_x_entry;
    NT_Entry _odom_y_entry;
    NT_Entry _odom_t_entry;
    NT_Entry _odom_vx_entry;
    NT_Entry _odom_vy_entry;
    NT_Entry _odom_vt_entry;
    NT_Entry _odom_update_entry;

    // ping entries
    NT_Entry _ping_entry;
    NT_Entry _ping_return_entry;

    // reset odom entries
    NT_Entry _set_odom_x_entry;
    NT_Entry _set_odom_y_entry;
    NT_Entry _set_odom_t_entry;
    NT_Entry _set_odom_update_entry;

    // waypoint entries
    NT_Entry _goal_status_entry;
    NT_Entry _goal_status_update_entry;

    // robot global pose entries
    NT_Entry _global_x_entry;
    NT_Entry _global_y_entry;
    NT_Entry _global_t_entry;
    NT_Entry _global_update_entry;

    // imu entries
    NT_Entry _imu_ax_entry;
    NT_Entry _imu_ay_entry;
    NT_Entry _imu_az_entry;
    NT_Entry _imu_gx_entry;
    NT_Entry _imu_gy_entry;
    NT_Entry _imu_gz_entry;
    NT_Entry _imu_r_entry;
    NT_Entry _imu_p_entry;
    NT_Entry _imu_y_entry;
    NT_Entry _imu_update_entry;

    // match entries
    NT_Entry _match_time_entry;
    NT_Entry _is_auto_entry;
    NT_Entry _team_color_entry;
    NT_Entry _match_update_entry;

    // pose estimate entries
    NT_Entry _pose_est_x_entry;
    NT_Entry _pose_est_y_entry;
    NT_Entry _pose_est_t_entry;
    NT_Entry _pose_est_update_entry;

    // waypoint entries
    NT_Entry _waypoint_is_continuous_entry;
    NT_Entry _waypoint_ignore_orientation_entry;
    NT_Entry _waypoint_intermediate_tolerance_entry;
    NT_Entry _waypoint_ignore_obstacles_entry;
    NT_Entry _waypoint_ignore_walls_entry;
    NT_Entry _waypoint_interruptable_by_entry;
    NT_Entry _waypoint_goal_x_entry;
    NT_Entry _waypoint_goal_y_entry;
    NT_Entry _waypoint_goal_t_entry;
    NT_Entry _waypoint_goal_name_entry;

    // waypoint plan entries
    NT_Entry _exec_waypoint_plan_entry;
    NT_Entry _exec_waypoint_plan_update_entry;
    NT_Entry _reset_waypoint_plan_entry;
    NT_Entry _cancel_waypoint_plan_entry;

    // velocity command entries
    NT_Entry _cmd_vel_x_entry;
    NT_Entry _cmd_vel_y_entry;
    NT_Entry _cmd_vel_t_entry;
    NT_Entry _cmd_vel_update_entry;

    // Members
    ros::Timer _ping_timer;
    ros::Duration _cmd_vel_timeout;
    ros::Timer _joint_timer;

    ros::Time _prev_twist_timestamp;
    double _twist_cmd_vx, _twist_cmd_vy, _twist_cmd_vt;

    tj2_waypoints::WaypointArray _waypoints;
    actionlib::SimpleActionClient<tj2_waypoints::FollowPathAction> *_waypoints_action_client;
    GoalStatus _currentGoalStatus;
    GoalStatus _prevPollStatus;


    // Messages
    nav_msgs::Odometry _odom_msg;
    sensor_msgs::Imu _imu_msg;
    vector<std_msgs::Float64*>* _raw_joint_msgs;

    // Publishers
    tf2_ros::TransformBroadcaster _tf_broadcaster;
    ros::Publisher _odom_pub;
    ros::Publisher _ping_pub;
    ros::Publisher _imu_pub;
    vector<ros::Publisher>* _raw_joint_pubs;
    ros::Publisher _match_time_pub, _autonomous_pub, _team_color_pub;
    ros::Publisher _pose_estimate_pub;

    // Subscribers
    ros::Subscriber _twist_sub;
    tf::TransformListener _tf_listener;

    // Service Servers
    ros::ServiceServer _odom_reset_srv;

    // NT publishers
    void publish_cmd_vel();
    void publish_goal_status();
    void publish_robot_global_pose();
    void publish_joints();

    // Subscription callbacks
    void twist_callback(const geometry_msgs::TwistConstPtr& msg);

    // Service callbacks
    bool odom_reset_callback(tj2_networktables::OdomReset::Request &req, tj2_networktables::OdomReset::Response &resp);

    // NT callbacks
    void odom_callback(const nt::EntryNotification& event);
    void ping_callback(const nt::EntryNotification& event);
    void imu_callback(const nt::EntryNotification& event);
    void joint_callback(size_t joint_index);
    void match_callback(const nt::EntryNotification& event);
    void pose_estimate_callback(const nt::EntryNotification& event);

    void create_waypoint(size_t index);
    void exec_waypoint_plan_callback(const nt::EntryNotification& event);
    void reset_waypoint_plan_callback(const nt::EntryNotification& event);
    void cancel_waypoint_plan_callback(const nt::EntryNotification& event);

    // Timer callbacks
    void ping_timer_callback(const ros::TimerEvent& event);
    void joint_timer_callback(const ros::TimerEvent& event);

    // NT helpers
    double get_double(NT_Entry entry, double default_value = 0.0);
    bool get_boolean(NT_Entry entry, bool default_value = false);
    string get_string(NT_Entry entry, string default_value = "");

    // Other helpers
    void add_joint_pub(string name);
    double get_time();
    vector<double> get_double_list_param(string name, size_t length);

    // Waypoint control
    void set_goal_status(GoalStatus status);
    void send_waypoints();
    void cancel_waypoint_goal();
    void reset_waypoints();
    tj2_waypoints::Waypoint make_waypoint_from_nt(size_t index);
    void add_waypoint(tj2_waypoints::Waypoint waypoint);

    // Main loop methods
    void loop();

public:
    TJ2NetworkTables(ros::NodeHandle* nodehandle);
    int run();
};