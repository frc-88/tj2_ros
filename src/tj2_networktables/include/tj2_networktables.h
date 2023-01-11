#pragma once

#include <string>
#include <fstream>
#include <iostream>
#include <math.h>

#include <boost/array.hpp>
#include <vector>
#include <boost/range/algorithm.hpp>
#include <iostream>
#include <cassert>

#include "ntcore.h"

#include "ros/ros.h"
#include "ros/console.h"

#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose.h"
#include "vision_msgs/Detection3DArray.h"
#include "sensor_msgs/LaserScan.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "apriltag_ros/AprilTagDetection.h"

#include "tj2_interfaces/Waypoint.h"
#include "tj2_interfaces/WaypointArray.h"

#include "tj2_interfaces/OdomReset.h"
#include "tj2_interfaces/NTEntry.h"
#include "tj2_interfaces/NavX.h"

#include "tj2_interfaces/OdomReset.h"
#include "tj2_interfaces/NTEntry.h"
#include "tj2_interfaces/NTEntryString.h"

#include "tj2_interfaces/ZoneInfo.h"
#include "tj2_interfaces/ZoneInfoArray.h"
#include "tj2_interfaces/NoGoZones.h"

#include "networktables/EntryListenerFlags.h"

using namespace std;


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
public:
    TJ2NetworkTables(ros::NodeHandle* nodehandle);
    int run();
protected:
    ros::NodeHandle nh;  // ROS node handle

    // NT entries
    NT_Inst _nt;
    string _base_key;

    // NT helpers
    double get_double(NT_Entry entry, double default_value = 0.0);
    vector<double> get_double_array(NT_Entry entry);
    vector<string> get_string_array(NT_Entry entry);
    bool get_boolean(NT_Entry entry, bool default_value = false);
    string get_string(NT_Entry entry, string default_value = "");
    NT_Entry get_entry(string path);

    // Main loop methods
    void loop();
private:
    // Parameters
    int _nt_port;
    double _update_interval;

    bool _publish_odom_tf;
    string _base_frame;
    string _odom_frame;
    string _map_frame;
    string _imu_frame;

    double _cmd_vel_timeout_param;
    double _odom_timeout_param;
    double _min_linear_cmd;
    double _min_angular_z_cmd;
    double _zero_epsilon;
    
    double _laser_angle_interval_rad;
    double _laser_angle_fan_rad;

    double _pose_estimate_x_std, _pose_estimate_y_std, _pose_estimate_theta_std_deg;
    string _pose_estimate_frame_id;

    std::vector<std::string> _joint_names;

    std::vector<double> _odom_covariance;
    std::vector<double> _twist_covariance;
    std::vector<double> _imu_orientation_covariance;
    std::vector<double> _imu_angular_velocity_covariance;
    std::vector<double> _imu_linear_acceleration_covariance;

    string _classes_path;

    // odom entries
    NT_Entry _odom_entry;
    NT_Entry _global_entry;
    NT_Entry _imu_entry;
    NT_Entry _pose_est_entry;
    NT_Entry _cmd_vel_entry;

    // ping entries
    NT_Entry _ping_entry;
    NT_Entry _ping_return_entry;

    // match entries
    NT_Entry _match_time_entry;
    NT_Entry _is_auto_entry;
    NT_Entry _team_color_entry;

    // velocity command entries
    NT_Entry _field_relative_entry;

    // joint entries
    NT_Entry _joint_states_entry;
    NT_Entry _joint_commands_entry;

    // waypoint entries
    NT_Entry _waypoint_names_entry;
    NT_Entry _waypoint_xs_entry;
    NT_Entry _waypoint_ys_entry;
    NT_Entry _waypoint_ts_entry;

    // laser scan entries
    NT_Entry _laser_entry_xs;
    NT_Entry _laser_entry_ys;

    // zone entries
    NT_Entry _zones_valid_entry;
    NT_Entry _zone_names_entry;
    NT_Entry _zone_nearest_x_entry;
    NT_Entry _zone_nearest_y_entry;
    NT_Entry _zone_distance_entry;
    NT_Entry _zone_is_inside_entry;
    NT_Entry _zone_is_nogo_entry;
    NT_Entry _nogo_zones_names_entry;
    NT_Entry _nogo_zones_update_entry;

    // Members
    ros::Timer _ping_timer;
    ros::Duration _cmd_vel_timeout;
    ros::Timer _joint_timer;
    ros::Duration _odom_timeout;

    ros::Time _prev_twist_timestamp;
    ros::Time _prev_odom_timestamp;
    ros::Time _prev_imu_timestamp;

    vector<double> _twist_cmd;
    vector<double> _global_pose;

    vector<string> _waypoint_names;
    vector<double> _waypoint_xs;
    vector<double> _waypoint_ys;
    vector<double> _waypoint_ts;

    vector<double> _joint_commands;

    tj2_interfaces::WaypointArray _waypoints;
    vector<string> _class_names;

    vector<double> _laser_scan_ranges;
    vector<double> _laser_scan_xs;
    vector<double> _laser_scan_ys;

    std::map<string, int> _detection_counter;

    // Messages
    nav_msgs::Odometry _odom_msg;
    tj2_interfaces::NavX _imu_msg;
    vector<std_msgs::Float64*>* _raw_joint_msgs;

    // Publishers
    tf2_ros::TransformBroadcaster _tf_broadcaster;
    ros::Publisher _odom_pub;
    ros::Publisher _ping_pub;
    ros::Publisher _imu_pub;
    vector<ros::Publisher>* _raw_joint_pubs;
    ros::Publisher _match_time_pub, _autonomous_pub, _team_color_pub;
    ros::Publisher _pose_estimate_pub;
    ros::Publisher _pose_reset_pub;
    ros::Publisher _nogo_zone_pub;

    // Subscribers
    ros::Subscriber _twist_sub;
    ros::Subscriber _nt_passthrough_sub;
    ros::Subscriber _nt_passthrough_string_sub;
    ros::Subscriber _waypoints_sub;
    ros::Subscriber _detections_sub;
    ros::Subscriber _field_relative_sub;
    ros::Subscriber _laser_sub;
    ros::Subscriber _zones_sub;
    ros::Subscriber _tags_sub;

    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener;
    vector<ros::Subscriber>* _raw_joint_subs;

    // NT publishers
    void publish_cmd_vel();
    void publish_robot_global_pose();
    void publish_detection_count(string name, int count);
    void publish_detection(string name, int index, geometry_msgs::PoseStamped pose);

    // NT subscribers
    void publish_joint(size_t joint_index, double joint_position);
    void publish_odom();
    void publish_imu();

    // Subscription callbacks
    void twist_callback(const geometry_msgs::TwistConstPtr& msg);
    void nt_passthrough_callback(const tj2_interfaces::NTEntryConstPtr& msg);
    void nt_passthrough_string_callback(const tj2_interfaces::NTEntryStringConstPtr& msg);
    void waypoints_callback(const tj2_interfaces::WaypointArrayConstPtr& msg);
    void detections_callback(const vision_msgs::Detection3DArrayConstPtr& msg);
    void joint_command_callback(const std_msgs::Float64ConstPtr& msg, int joint_index);
    void field_relative_callback(const std_msgs::BoolConstPtr& msg);
    void scan_callback(const sensor_msgs::LaserScanConstPtr& msg);
    void zones_info_callback(const tj2_interfaces::ZoneInfoArrayConstPtr& msg);
    void tags_callback(const apriltag_ros::AprilTagDetectionArrayConstPtr& msg);

    // NT callbacks
    void odom_callback(const nt::EntryNotification& event);
    void ping_callback(const nt::EntryNotification& event);
    void imu_callback(const nt::EntryNotification& event);
    void match_callback(const nt::EntryNotification& event);
    void pose_estimate_callback(const nt::EntryNotification& event);
    void joint_state_callback(const nt::EntryNotification& event);

    void nogo_zones_callback(const nt::EntryNotification& event);

    // Timer callbacks
    void ping_timer_callback(const ros::TimerEvent& event);

    // Other helpers
    void add_joint(string name);
    double get_time();
    vector<double> get_double_list_param(string name, size_t length);
    string get_label(int obj_id);
    int get_index(int obj_id);
    std::vector<std::string> load_label_names(const string& path);
    
};