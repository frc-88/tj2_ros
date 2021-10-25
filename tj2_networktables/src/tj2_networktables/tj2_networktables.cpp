#include "tj2_networktables/tj2_networktables.h"

TJ2NetworkTables::TJ2NetworkTables(ros::NodeHandle* nodehandle) :
    nh(*nodehandle)
{
    ros::param::param<string>("~nt_host", _nt_host, "127.0.0.1");
    ros::param::param<int>("~nt_port", _nt_port, 1735);

    ros::param::param<double>("~remote_linear_units_conversion", _remote_linear_units_conversion, 0.3048);
    ros::param::param<double>("~remote_angular_units_conversion", _remote_angular_units_conversion, M_PI / 180.0);
    ros::param::param<int>("~num_modules", _num_modules, 4);
    ros::param::param<bool>("~fms_flag_ignored", _fms_flag_ignored, true);

    _nt = nt::GetDefaultInstance();
    nt::AddLogger(_nt,
                [](const nt::LogMessage& msg) {
                //   std::fputs(msg.message.c_str(), stderr);
                //   std::fputc('\n', stderr);
                    ROS_DEBUG("[NT]:\t %s", msg.message.c_str());
                },
                0, UINT_MAX
    );
    nt::StartClient(_nt, _nt_host.c_str(), _nt_port);
    ros::Duration(2.0).sleep(); // wait for table to populate

    _base_key = "/swerveLibrary/";
    _module_table_key = "modules/";
    _driver_station_table_key = "DriverStation/";
    _client_table_key = "ROS/";
    _set_odom_table_key = _client_table_key + "setOdom/";

    _remote_timestamp_entry = nt::GetEntry(_nt, _base_key + "timestamp");

    _fms_attached_entry = nt::GetEntry(_nt, _base_key + _driver_station_table_key + "isFMSAttached");
    _is_autonomous_entry = nt::GetEntry(_nt, _base_key + _driver_station_table_key + "isAutonomous");
    _match_time_entry = nt::GetEntry(_nt, _base_key + _driver_station_table_key + "getMatchTime");

    _set_odom_time_entry = nt::GetEntry(_nt, _base_key + _set_odom_table_key + "timestamp");
    _set_odom_y_entry = nt::GetEntry(_nt, _base_key + _set_odom_table_key + "xPosition");
    _set_odom_x_entry = nt::GetEntry(_nt, _base_key + _set_odom_table_key + "yPosition");
    _set_odom_t_entry = nt::GetEntry(_nt, _base_key + _set_odom_table_key + "theta");
    nt::SetEntryFlags(_set_odom_time_entry, NT_PERSISTENT);
    nt::SetEntryFlags(_set_odom_y_entry, NT_PERSISTENT);
    nt::SetEntryFlags(_set_odom_x_entry, NT_PERSISTENT);
    nt::SetEntryFlags(_set_odom_t_entry, NT_PERSISTENT);

    _prev_remote_timestamp = 0.0;
    _local_start_time = 0.0;
    _remote_start_time = 0.0;

    _module_pubs = new vector<ros::Publisher>();
    _module_msgs = new vector<tj2_networktables::SwerveModule*>();
    _module_entries = new vector<ModuleEntries_t*>();
    for (int index = 0; index < _num_modules; index++)
    {
        string module_name = std::to_string(index);
        _module_msgs->push_back(new tj2_networktables::SwerveModule());

        _module_pubs->push_back(
            nh.advertise<tj2_networktables::SwerveModule>("swerve_modules/" + module_name, 50)
        );

        ModuleEntries_t* entries = new ModuleEntries_t;
        init_module_entries(entries, index);
        _module_entries->push_back(entries);
    }

    _match_time_pub = nh.advertise<std_msgs::Float64>("match_time", 5);
    _is_autonomous_pub = nh.advertise<std_msgs::Bool>("is_autonomous", 5);

    _odom_reset_srv = nh.advertiseService("odom_reset_service", &TJ2NetworkTables::odom_reset_callback, this);

    ROS_INFO("tj2_networktables init complete");
}

// -----
// Modules
// -----


void TJ2NetworkTables::init_module_entries(ModuleEntries_t* entries, int module_index)
{
    string module_key = std::to_string(module_index) + "/";
    
    entries->wheel_velocity = nt::GetEntry(_nt, _base_key + _module_table_key + module_key + "wheelVelocity");
    entries->azimuth_position = nt::GetEntry(_nt, _base_key + _module_table_key + module_key + "azimuthPosition");
    entries->azimuth_velocity = nt::GetEntry(_nt, _base_key + _module_table_key + module_key + "azimuthVelocity");
    
    entries->command_wheel_velocity = nt::GetEntry(_nt, _base_key + _module_table_key + module_key + "commandWheelVelocity");
    entries->command_azimuth_position = nt::GetEntry(_nt, _base_key + _module_table_key + module_key + "commandAzimuthPosition");
    entries->command_azimuth_velocity = nt::GetEntry(_nt, _base_key + _module_table_key + module_key + "commandAzimuthVelocity");
    
    entries->target_wheel_velocity = nt::GetEntry(_nt, _base_key + _module_table_key + module_key + "targetWheelVelocity");
    entries->target_azimuth_position = nt::GetEntry(_nt, _base_key + _module_table_key + module_key + "targetAzimuthPosition");
    entries->target_azimuth_velocity = nt::GetEntry(_nt, _base_key + _module_table_key + module_key + "targetAzimuthVelocity");
    
    entries->location_x = nt::GetEntry(_nt, _base_key + _module_table_key + module_key + "locationX");
    entries->location_y = nt::GetEntry(_nt, _base_key + _module_table_key + module_key + "locationY");

    entries->motor_lo_0 = new MotorEntries_t;
    entries->motor_hi_1 = new MotorEntries_t;
    init_motor_entries(entries->motor_lo_0, module_index, "motor0/");
    init_motor_entries(entries->motor_hi_1, module_index, "motor1/");
}

void TJ2NetworkTables::init_motor_entries(MotorEntries_t* entries, int module_index, string motor_key)
{
    string module_key = std::to_string(module_index) + "/";
    entries->velocity = nt::GetEntry(_nt, _base_key + _module_table_key + module_key + motor_key + "velocity");
    entries->command_velocity = nt::GetEntry(_nt, _base_key + _module_table_key + module_key + motor_key + "commandVelocity");
    entries->command_voltage = nt::GetEntry(_nt, _base_key + _module_table_key + module_key + motor_key + "commandVoltage");
    entries->current_draw = nt::GetEntry(_nt, _base_key + _module_table_key + module_key + motor_key + "currentDraw");
}

void TJ2NetworkTables::publish_modules()
{
    for (int index = 0; index < _num_modules; index++)
    {
        ModuleEntries_t* entries = _module_entries->at(index);
        tj2_networktables::SwerveModule* msg = _module_msgs->at(index);
        ros::Publisher pub = _module_pubs->at(index);

        msg->wheel_velocity = _remote_linear_units_conversion * getDouble(entries->wheel_velocity);
        msg->azimuth_position = _remote_angular_units_conversion * getDouble(entries->azimuth_position);
        msg->azimuth_velocity = _remote_angular_units_conversion * getDouble(entries->azimuth_velocity);
        
        msg->command_wheel_velocity = _remote_linear_units_conversion * getDouble(entries->command_wheel_velocity);
        msg->command_azimuth_position = _remote_angular_units_conversion * getDouble(entries->command_azimuth_position);
        msg->command_azimuth_velocity = _remote_angular_units_conversion * getDouble(entries->command_azimuth_velocity);
        
        msg->target_wheel_velocity = _remote_linear_units_conversion * getDouble(entries->target_wheel_velocity);
        msg->target_azimuth_position = _remote_angular_units_conversion * getDouble(entries->target_azimuth_position);
        msg->target_azimuth_velocity = _remote_angular_units_conversion * getDouble(entries->target_azimuth_velocity);

        msg->location_x = _remote_linear_units_conversion * getDouble(entries->location_x);
        msg->location_y = _remote_linear_units_conversion * getDouble(entries->location_y);
        
        update_motor_module(msg->motor_lo_0, entries->motor_lo_0);
        update_motor_module(msg->motor_hi_1, entries->motor_hi_1);

        pub.publish(*msg);
    }
}

void TJ2NetworkTables::update_motor_module(tj2_networktables::SwerveMotor& msg, MotorEntries_t* motor_entries)
{
    msg.velocity = getDouble(motor_entries->velocity);
    msg.command_velocity = getDouble(motor_entries->command_velocity);
    msg.command_voltage = getDouble(motor_entries->command_voltage);
    msg.current_draw = getDouble(motor_entries->current_draw);
}

// -----
// FRC FMS
// -----

void TJ2NetworkTables::publish_fms()
{
    bool is_fms_attached = getBoolean(_fms_attached_entry, false);
    bool is_autonomous = getBoolean(_is_autonomous_entry, true);

    std_msgs::Float64 match_time_msg;
    if (_fms_flag_ignored || is_fms_attached)
    {
        match_time_msg.data = getDouble(_match_time_entry, -1.0);

        std_msgs::Bool is_auto_msg;
        is_auto_msg.data = is_autonomous;
        _is_autonomous_pub.publish(is_auto_msg);
    }
    else {
        match_time_msg.data = -1.0;
    }
    _match_time_pub.publish(match_time_msg);
}


// -----
// Time
// -----


double TJ2NetworkTables::get_remote_time() {
    // Gets RoboRIO's timestamp based on the networktables entry in seconds
    auto timestamp_val = nt::GetEntryValue(_remote_timestamp_entry);
    if (!timestamp_val) {
        ROS_WARN_THROTTLE(5, "Remote timestamp entry doesn't exist!");
        return 0.0;
    }
    if (!timestamp_val->IsDouble()) {
        ROS_WARN_THROTTLE(5, "Remote timestamp entry is not a double");
        return 0.0;
    }
    return timestamp_val->GetDouble() * 1E-6;
}

double TJ2NetworkTables::get_local_time()
{
    return ros::Time::now().toSec();
}

void TJ2NetworkTables::check_time_offsets(double remote_timestamp)
{
    if (remote_timestamp < _prev_remote_timestamp)  // remote has restarted
    {
        _remote_start_time = remote_timestamp;
        _local_start_time = get_local_time();
        ROS_INFO("Remote clock jumped back. Resetting offset");
    }
    _prev_remote_timestamp = remote_timestamp;
}
    

double TJ2NetworkTables::get_local_time_as_remote()
{
    double local_timestamp = get_local_time();
    double remote_timestamp = get_remote_time();
    check_time_offsets(remote_timestamp);
    double remote_time = local_timestamp - _local_start_time + _remote_start_time;
    remote_time *= 1E6;
    return remote_time;
}
double TJ2NetworkTables::get_remote_time_as_local()
{
    // Gets the RoboRIO's time relative to ROS time epoch
    double remote_timestamp = get_remote_time();
    check_time_offsets(remote_timestamp);
    return remote_timestamp - _remote_start_time + _local_start_time;
}

void TJ2NetworkTables::init_remote_time()
{
    _remote_start_time = get_remote_time();
    _local_start_time = get_local_time();
}

// -----
// NT helpers
// -----

double TJ2NetworkTables::getDouble(NT_Entry entry, double default_value)
{
    auto value = nt::GetEntryValue(entry);
    if (value && value->IsDouble()) {
        return value->GetDouble();
    }
    else {
        return default_value;
    }
}

double TJ2NetworkTables::getBoolean(NT_Entry entry, bool default_value)
{
    auto value = nt::GetEntryValue(entry);
    if (value && value->IsBoolean()) {
        return value->GetBoolean();
    }
    else {
        return default_value;
    }
}

// -----
// Services
// -----
bool TJ2NetworkTables::odom_reset_callback(tj2_networktables::OdomReset::Request &req, tj2_networktables::OdomReset::Response &resp)
{
    nt::SetEntryValue(_set_odom_time_entry, nt::Value::MakeDouble(get_local_time_as_remote()));
    nt::SetEntryValue(_set_odom_x_entry, nt::Value::MakeDouble(req.x));
    nt::SetEntryValue(_set_odom_y_entry, nt::Value::MakeDouble(req.y));
    nt::SetEntryValue(_set_odom_t_entry, nt::Value::MakeDouble(req.t));
    ROS_INFO("Resetting odometry to x: %0.3f, y: %0.3f, theta: %0.3f", req.x, req.y, req.t);
    resp.resp = true;
    return true;
}


void TJ2NetworkTables::loop()
{
    if (_remote_start_time == 0.0) {
        init_remote_time();
    }
    publish_modules();
    publish_fms();
}

int TJ2NetworkTables::run()
{
    ros::Rate clock_rate(100);  // Hz

    int exit_code = 0;
    while (ros::ok())
    {
        // let ROS process any events
        ros::spinOnce();
        clock_rate.sleep();

        try {
            loop();
        }
        catch (exception& e) {
            ROS_ERROR_STREAM("Exception in main loop: " << e.what());
            exit_code = 1;
            break;
        }
    }
    return exit_code;
}
