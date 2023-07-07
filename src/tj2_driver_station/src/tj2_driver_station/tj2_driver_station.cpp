#include "tj2_driver_station/tj2_driver_station.h"


TJ2DriverStation::TJ2DriverStation(ros::NodeHandle* nodehandle) :
    nh(*nodehandle)
{
    ros::param::param<string>("~frc_robot_address", _frc_robot_address, "10.0.88.2");
    ros::param::param<int>("~start_mode", _start_mode, 0);
    ros::param::param<double>("~alive_time_threshold", _alive_time_threshold_param, 5.0);
    ros::param::param<double>("~disable_time_threshold", _disable_time_threshold_param, 10.0);
    ros::param::param<string>("~alliance", _alliance, "red");
    ros::param::param<string>("~position", _position, "1");

    if (_start_mode == NOMODE) {
        _start_mode = DISABLED;
    }
    _alive_time_threshold = ros::Duration(_alive_time_threshold_param);
    _disable_time_threshold = ros::Duration(_disable_time_threshold_param);
    _heartbeat_time = ros::Time(0);
    _current_mode = NOMODE;

    status_pub = nh.advertise<tj2_driver_station::RobotStatus>("robot_status", 20);

    robot_mode_srv = nh.advertiseService("robot_mode", &TJ2DriverStation::robot_mode_callback, this);
    alliance_srv = nh.advertiseService("alliance", &TJ2DriverStation::alliance_callback, this);

    enable_sub = nh.subscribe<std_msgs::Bool>("motor_enable", 1, &TJ2DriverStation::motor_enable_callback, this);

    _frc_protocol = DS_GetProtocolFRC_2020();

    ROS_INFO("Connecting to robot at %s", _frc_robot_address.c_str());
    ROS_INFO("tj2_driver_station init done");
}

bool TJ2DriverStation::robot_mode_callback(tj2_driver_station::SetRobotMode::Request &req, tj2_driver_station::SetRobotMode::Response &resp)
{
    if (!_is_robot_connected) {
        resp.success = false;
        resp.message = "Robot isn't connected! Can't set mode.";
        return true;
    }
    if (!_is_code_running) {
        resp.success = false;
        resp.message = "Robot code isn't running! Can't set mode.";
        return true;
    }
    if (set_mode((RobotMode)req.mode)) {
        resp.success = true;
        resp.message = "";
    }
    else {
        resp.success = false;
        resp.message = "Failed to set mode!";
    }
    return true;
}

bool TJ2DriverStation::alliance_callback(tj2_driver_station::SetAlliance::Request &req, tj2_driver_station::SetAlliance::Response &resp)
{
    if (!set_alliance(req.alliance)) {
        resp.success = false;
        return false;
    }
    if (!set_position(req.position)) {
        resp.success = false;
        return false;
    }
    resp.success = true;
    return true;
}


void TJ2DriverStation::motor_enable_callback(const std_msgs::BoolConstPtr& msg) {
    if (msg->data) {
        set_mode(TELEOP);
    }
    else {
        set_mode(DISABLED);
    }
}

bool TJ2DriverStation::set_mode(RobotMode mode)
{
    _requested_mode = mode;
    if (!_is_robot_connected) {
        ROS_WARN_THROTTLE(1.0, "Robot isn't connected! Can't set mode.");
        return false;
    }
    if (!_is_code_running) {
        ROS_WARN_THROTTLE(1.0, "Robot code isn't running! Can't set mode.");
        return false;
    }
    if (_current_mode == mode) {
        return true;
    }
    switch (mode) {
        case NOMODE:
        case DISABLED:
            DS_SetRobotEnabled(0);
            DS_SetEmergencyStopped(0);
            break;
        case ESTOP:
            DS_SetEmergencyStopped(1);
            break;
        case TELEOP:
            DS_SetRobotEnabled(1);
            DS_SetControlMode(DS_CONTROL_TELEOPERATED);
            break;
        case AUTONOMOUS:
            DS_SetRobotEnabled(1);
            DS_SetControlMode(DS_CONTROL_AUTONOMOUS);
            break;
        case TEST:
            DS_SetRobotEnabled(1);
            DS_SetControlMode(DS_CONTROL_TEST);
            break;
        default:
            ROS_WARN("Invalid mode: %d", mode);
            return false;
    }
    ROS_INFO("Set mode to %d", mode);
    _current_mode = mode;
    status_msg.mode = mode;
    return true;
}

bool TJ2DriverStation::set_alliance(string alliance)
{
    _alliance = alliance;
    if (_alliance.compare("red") == 0) {
        DS_SetAlliance(DS_ALLIANCE_RED);
    }
    else if (_alliance.compare("blue") == 0) {
        DS_SetAlliance(DS_ALLIANCE_BLUE);
    }
    else {
        ROS_ERROR_STREAM("Failed to set alliance color: " << _alliance);
        return false;
    }
    return true;
}

bool TJ2DriverStation::set_position(string position)
{
    _position = position;
    if (_position.compare("1") == 0) {
        DS_SetPosition(DS_POSITION_1);
    }
    else if (_position.compare("2") == 0) {
        DS_SetPosition(DS_POSITION_2);
    }
    else if (_position.compare("3") == 0) {
        DS_SetPosition(DS_POSITION_3);
    }
    else {
        ROS_ERROR_STREAM("Failed to set alliance position: " << _position);
        return false;
    }
    return true;
}


void TJ2DriverStation::process_events()
{
    if (!DS_Initialized()) {
        _is_robot_connected = false;
        _is_code_running = false;
        status_msg.mode = NOMODE;
        _current_mode = NOMODE;
    }
    else {
        DS_Event event;
        while (DS_PollEvent(&event))
        {
            _heartbeat_time = ros::Time::now();
            switch (event.type)
            {
                case DS_JOYSTICK_COUNT_CHANGED: ROS_DEBUG("Number of joysticks: %d", DS_GetJoystickCount()); break;
                case DS_NETCONSOLE_NEW_MESSAGE:  break;
                case DS_ROBOT_VOLTAGE_CHANGED: 
                    status_msg.voltage = event.robot.voltage;
                    ROS_DEBUG("FRC Robot Voltage: %f", event.robot.voltage);
                    break;
                case DS_ROBOT_CAN_UTIL_CHANGED: 
                    status_msg.can_util = event.robot.can_util;
                    ROS_DEBUG("FRC Robot CAN Util: %d", event.robot.can_util);
                    break;
                case DS_ROBOT_CPU_INFO_CHANGED: 
                    status_msg.cpu_usage = event.robot.cpu_usage;
                    ROS_DEBUG("FRC Robot CPU: %d", event.robot.cpu_usage);
                    break;
                case DS_ROBOT_RAM_INFO_CHANGED: 
                    status_msg.ram_usage = event.robot.ram_usage;
                    ROS_DEBUG("FRC Robot RAM: %d", event.robot.ram_usage);
                    break;
                case DS_ROBOT_DISK_INFO_CHANGED: 
                    status_msg.disk_usage = event.robot.disk_usage;
                    ROS_DEBUG("FRC Robot Disk: %d", event.robot.disk_usage);
                    break;
                case DS_STATUS_STRING_CHANGED: 
                    status_msg.status = DS_GetStatusString();
                    ROS_DEBUG("FRC Robot status: %s", DS_GetStatusString());
                    break;
                case DS_ROBOT_COMMS_CHANGED:
                    _is_robot_connected = event.robot.connected;
                    status_msg.is_connected = _is_robot_connected;
                    ROS_INFO("FRC Robot comms: %d", event.robot.connected);
                    break;
                case DS_ROBOT_CODE_CHANGED:
                    _is_code_running = event.robot.code;
                    status_msg.has_code = _is_code_running;
                    ROS_INFO("FRC Robot code: %d", event.robot.code);
                    break;
                default:
                    break;
            }
        }
    }
    
    status_pub.publish(status_msg);
}
bool TJ2DriverStation::is_alive()
{
    return ros::Time::now() - _heartbeat_time < _alive_time_threshold;
}

bool TJ2DriverStation::is_connected()
{
    return _is_robot_connected;
}


void TJ2DriverStation::init()
{
    if (DS_Initialized()) {
        return;
    }
    ROS_INFO("Initializing robot connection");
    DS_Init();
    DS_SetCustomRobotAddress(_frc_robot_address.c_str());
    DS_ConfigureProtocol(&_frc_protocol);
    set_alliance(_alliance);
    set_position(_position);
    
    _requested_mode = (RobotMode)_start_mode;
    _current_mode = NOMODE;

    // Wait a bit for events to stream in
    ros::Time init_time = ros::Time::now();
    ros::Duration timeout = ros::Duration(5.0);
    while (ros::ok()) 
    {
        ros::spinOnce();
        if (ros::Time::now() - init_time > timeout) {
            break;
        }
        process_events();
        DS_Sleep(20);
        
        if (is_connected()) {
            break;
        }
    }
}

void TJ2DriverStation::close()
{
    if (!DS_Initialized()) {
        ROS_INFO("Robot is already closed");
        return;
    }
    ROS_INFO("Closing robot connection");
    DS_Close();
}

int TJ2DriverStation::run()
{
    init();
    while (ros::ok())
    {
        ros::spinOnce();
        process_events();
        DS_Sleep(20);
        
        if (is_connected()) {
            if (_current_mode != _requested_mode) {
                set_mode(_requested_mode);
            }
        }
        if (_current_mode != NOMODE && (!_is_robot_connected || !_is_code_running)) {
            _current_mode = NOMODE;
            set_mode(NOMODE);
        }
    }
    close();
    return 0;
}
