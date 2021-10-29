#pragma once

#include "ros/ros.h"
#include "ros/console.h"

#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/Imu.h"
#include "tj2_networktables/OdomReset.h"

#include "ntcore.h"

using namespace std;

// NT callbacks
void ping_callback(void* data, const NT_EntryNotification* event);


class TJ2NetworkTables {
private:
    ros::NodeHandle nh;  // ROS node handle

    // Parameters
    string _nt_host;
    int _nt_port;
    double _remote_linear_units_conversion;
    double _remote_angular_units_conversion;
    int _num_modules;
    bool _fms_flag_ignored;

    // Properties
    NT_Inst _nt;
    string _base_key;
    string _driver_station_table_key;
    string _client_table_key;
    string _set_odom_table_key;

    NT_Entry _remote_timestamp_entry;

    NT_Entry _fms_attached_entry;
    NT_Entry _is_autonomous_entry;
    NT_Entry _match_time_entry;

    NT_Entry _set_odom_time_entry;
    NT_Entry _set_odom_x_entry;
    NT_Entry _set_odom_y_entry;
    NT_Entry _set_odom_t_entry;

    double _prev_remote_timestamp;
    double _remote_start_time;
    double _local_start_time;

    // Messages
    tf2_ros::TransformBroadcaster _tf_broadcaster;
    ros::Publisher _match_time_pub;
    ros::Publisher _is_autonomous_pub;

    // Service Servers
    ros::ServiceServer _odom_reset_srv;

    // Service callbacks
    bool odom_reset_callback(tj2_networktables::OdomReset::Request &req, tj2_networktables::OdomReset::Response &resp);

    // Main loop methods
    void loop();
    void publish_fms();

    // time methods
    double get_remote_time();
    double get_local_time();
    void check_time_offsets(double remote_timestamp);
    double get_local_time_as_remote();
    double get_remote_time_as_local();
    void init_remote_time();

    double getDouble(NT_Entry entry, double default_value = 0.0);
    double getBoolean(NT_Entry entry, bool default_value = false);

public:
    TJ2NetworkTables(ros::NodeHandle* nodehandle);
    int run();
};
