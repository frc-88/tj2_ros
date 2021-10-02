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
#include "tj2_networktables/SwerveModule.h"
#include "tj2_networktables/SwerveMotor.h"
#include "tj2_networktables/OdomReset.h"

#include "ntcore.h"

using namespace std;

// NT callbacks
void ping_callback(void* data, const NT_EntryNotification* event);

typedef struct MotorEntries
{
    NT_Entry velocity;
    NT_Entry command_velocity;
    NT_Entry command_voltage;
    NT_Entry current_draw;
    
} MotorEntries_t;

typedef struct ModuleEntries
{
    NT_Entry wheel_velocity;
    NT_Entry azimuth_position;
    NT_Entry azimuth_velocity;
    NT_Entry command_wheel_velocity;
    NT_Entry command_azimuth_position;
    NT_Entry command_azimuth_velocity;
    NT_Entry target_wheel_velocity;
    NT_Entry target_azimuth_position;
    NT_Entry target_azimuth_velocity;
    NT_Entry location_x;
    NT_Entry location_y;

    MotorEntries_t* motor_lo_0;
    MotorEntries_t* motor_hi_1;
    
} ModuleEntries_t;



class TJ2NetworkTables {
private:
    ros::NodeHandle nh;  // ROS node handle

    // Parameters
    string _nt_host;
    int _nt_port;
    double _remote_linear_units_conversion;
    double _remote_angular_units_conversion;
    bool _publish_odom_tf;
    string _base_frame;
    string _odom_frame;
    string _imu_frame;
    double _cmd_vel_timeout;
    double _min_linear_x_cmd;
    double _min_linear_y_cmd;
    double _min_angular_z_cmd;
    double _zero_epsilon;
    int _num_modules;
    bool _fms_flag_ignored;

    // Properties
    NT_Inst _nt;
    string _base_key;
    string _odom_table_key;
    string _command_table_key;
    string _module_table_key;
    string _imu_table_key;
    string _driver_station_table_key;
    string _client_table_key;

    NT_Entry _remote_timestamp_entry;

    NT_Entry _x_entry;
    NT_Entry _y_entry;
    NT_Entry _theta_entry;
    NT_Entry _vx_entry;
    NT_Entry _vy_entry;
    NT_Entry _vt_entry;

    NT_Entry _cmd_time_entry;
    NT_Entry _cmd_speed_entry;
    NT_Entry _cmd_dir_entry;
    NT_Entry _cmd_rotate_entry;

    NT_Entry _heartbeat_time_entry;
    NT_Entry _heartbeat_ping_entry;
    NT_Entry _return_ping_entry;

    NT_Entry _imu_yaw_entry;
    NT_Entry _imu_vz_entry;
    NT_Entry _imu_ax_entry;
    NT_Entry _imu_ay_entry;

    vector<ModuleEntries_t*>* _module_entries;

    NT_Entry _fms_attached_entry;
    NT_Entry _is_autonomous_entry;
    NT_Entry _match_time_entry;

    double _prev_remote_timestamp;
    double _remote_start_time;
    double _local_start_time;

    double _start_x, _start_y, _start_theta;
    double _odom_x, _odom_y, _odom_theta;

    double _prev_twist_timestamp;
    double _twist_cmd_speed, _twist_cmd_dir, _twist_cmd_vt;

    double _prev_imu_timestamp;

    // Messages
    nav_msgs::Odometry _odom_msg;
    sensor_msgs::Imu _imu_msg;
    vector<tj2_networktables::SwerveModule*>* _module_msgs;

    // Publishers
    ros::Publisher _odom_pub;
    ros::Publisher _ping_pub;
    ros::Publisher _imu_pub;
    vector<ros::Publisher>* _module_pubs;
    tf2_ros::TransformBroadcaster _tf_broadcaster;
    ros::Publisher _match_time_pub;
    ros::Publisher _is_autonomous_pub;

    // Subscribers
    ros::Subscriber _twist_sub;

    // Service Servers
    ros::ServiceServer _odom_reset_srv;

    // Topic callbacks
    void twist_callback(const geometry_msgs::TwistConstPtr& msg);

    // Service callbacks
    bool odom_reset_callback(tj2_networktables::OdomReset::Request &req, tj2_networktables::OdomReset::Response &resp);

    // Main loop methods
    void loop();
    void get_ping_value();
    void publish_heartbeat();
    void publish_odom();
    void publish_cmd_vel(bool ignore_timeout = false);
    void publish_imu();
    void publish_modules();
    void publish_fms();

    // time methods
    double get_remote_time();
    double get_local_time();
    void check_time_offsets(double remote_timestamp);
    double get_local_time_as_remote();
    double get_remote_time_as_local();
    void init_remote_time();

    // Module methods
    void init_module_entries(ModuleEntries_t* entries, int module_index);
    void init_motor_entries(MotorEntries_t* entries, int module_index, string motor_key);
    void update_motor_module(tj2_networktables::SwerveMotor& msg, MotorEntries_t* motor_entries);

    double getDouble(NT_Entry entry, double default_value = 0.0);
    double getBoolean(NT_Entry entry, bool default_value = false);

public:
    void publish_ping(double ping_time);

    TJ2NetworkTables(ros::NodeHandle* nodehandle);
    int run();
};
