#pragma once

#include <exception>
#include <iostream>
#include <ctime>
#include <math.h>

#include "ros/ros.h"
#include "ros/console.h"

#include <libds/LibDS.h>

#include "tj2_driver_station/SetRobotMode.h"
#include "tj2_driver_station/RobotStatus.h"

using namespace std;

typedef enum {
    NOMODE=0,
    DISABLED=1,
    ESTOP=2,
    TELEOP=3,
    AUTONOMOUS=4,
    TEST=5
} RobotMode;

class TJ2DriverStation
{
private:
    ros::NodeHandle nh;  // ROS node handle

    // Launch parameters
    string _frc_robot_address;
    int _start_mode;
    double _alive_time_threshold_param;

    // Object properties
    bool _is_robot_connected;
    bool _is_code_running;
    RobotMode _current_mode;
    RobotMode _requested_mode;
    tj2_driver_station::RobotStatus status_msg;
    DS_Protocol _frc_protocol;
    ros::Duration _alive_time_threshold;
    ros::Time _heartbeat_time;

    // Subscribers

    // Publishers
    ros::Publisher status_pub;

    // Services
    ros::ServiceServer robot_mode_srv;

    // Callbacks
    bool robot_mode_callback(tj2_driver_station::SetRobotMode::Request &req, tj2_driver_station::SetRobotMode::Response &resp);

    void init();
    void close();
    bool is_connected();
    bool is_alive();
    void wait_for_connection();
    void process_events();
    bool set_mode(RobotMode mode);

public:
    TJ2DriverStation(ros::NodeHandle* nodehandle);
    int run();
};
