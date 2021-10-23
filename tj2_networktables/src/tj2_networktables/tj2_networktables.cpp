#include "tj2_networktables/tj2_networktables.h"

TJ2NetworkTables::TJ2NetworkTables(ros::NodeHandle* nodehandle) :
    nh(*nodehandle)
{
    ros::param::param<string>("~nt_host", _nt_host, "127.0.0.1");
    ros::param::param<int>("~nt_port", _nt_port, 1735);

    ros::param::param<double>("~remote_linear_units_conversion", _remote_linear_units_conversion, 0.3048);
    ros::param::param<double>("~remote_angular_units_conversion", _remote_angular_units_conversion, M_PI / 180.0);

    ros::param::param<bool>("~publish_odom_tf", _publish_odom_tf, true);
    ros::param::param<string>("~base_frame", _base_frame, "base_link");
    ros::param::param<string>("~odom_frame", _odom_frame, "odom");
    ros::param::param<string>("~imu_frame", _imu_frame, "imu");
    ros::param::param<double>("~cmd_vel_timeout", _cmd_vel_timeout, 0.5);
    ros::param::param<double>("~min_linear_x_cmd", _min_linear_x_cmd, 0.05);
    ros::param::param<double>("~min_linear_y_cmd", _min_linear_y_cmd, 0.05);
    ros::param::param<double>("~min_angular_z_cmd", _min_angular_z_cmd, 0.1);
    ros::param::param<double>("~zero_epsilon", _zero_epsilon, 0.001);
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

    _base_key = "/coprocessor/";
    _odom_table_key = "odometryState/";
    _command_table_key = "commands/";
    _imu_table_key = "gyro/";
    _driver_station_table_key = "DriverStation/";
    _client_table_key = "ROS/";
    _set_odom_table_key = _client_table_key + "setOdom/";

    _remote_timestamp_entry = nt::GetEntry(_nt, _base_key + "timestamp");

    _heartbeat_time_entry = nt::GetEntry(_nt, _base_key + _client_table_key + "timestamp");
    _heartbeat_ping_entry = nt::GetEntry(_nt, _base_key + _client_table_key + "ping");
    nt::SetEntryFlags(_heartbeat_time_entry, NT_PERSISTENT);
    nt::SetEntryFlags(_heartbeat_ping_entry, NT_PERSISTENT);
    
    string ping_response_key = _base_key + _client_table_key + "pingResponse";
    _return_ping_entry = nt::GetEntry(_nt, ping_response_key);
    // NT_AddEntryListener(_nt, ping_response_key.c_str(), ping_response_key.size(), this, &ping_callback, NT_NOTIFY_IMMEDIATE | NT_NOTIFY_UPDATE);

    _x_entry = nt::GetEntry(_nt, _base_key + _odom_table_key + "xPosition");
    _y_entry = nt::GetEntry(_nt, _base_key + _odom_table_key + "yPosition");
    _theta_entry = nt::GetEntry(_nt, _base_key + _odom_table_key + "theta");
    _vx_entry = nt::GetEntry(_nt, _base_key + _odom_table_key + "xVelocity");
    _vy_entry = nt::GetEntry(_nt, _base_key + _odom_table_key + "yVelocity");
    _vt_entry = nt::GetEntry(_nt, _base_key + _odom_table_key + "thetaVelocity");

    _cmd_time_entry = nt::GetEntry(_nt, _base_key + _command_table_key + "timestamp");
    _cmd_speed_entry = nt::GetEntry(_nt, _base_key + _command_table_key + "translationSpeed");
    _cmd_dir_entry = nt::GetEntry(_nt, _base_key + _command_table_key + "translationDirection");
    _cmd_rotate_entry = nt::GetEntry(_nt, _base_key + _command_table_key + "rotationVelocity");
    nt::SetEntryFlags(_cmd_time_entry, NT_PERSISTENT);
    nt::SetEntryFlags(_cmd_speed_entry, NT_PERSISTENT);
    nt::SetEntryFlags(_cmd_dir_entry, NT_PERSISTENT);
    nt::SetEntryFlags(_cmd_rotate_entry, NT_PERSISTENT);

    _imu_yaw_entry = nt::GetEntry(_nt, _base_key + _imu_table_key + "yaw");
    _imu_vz_entry = nt::GetEntry(_nt, _base_key + _imu_table_key + "yawRate");
    _imu_ax_entry = nt::GetEntry(_nt, _base_key + _imu_table_key + "accelX");
    _imu_ay_entry = nt::GetEntry(_nt, _base_key + _imu_table_key + "accelY");

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

    _ping_pub = nh.advertise<std_msgs::Float64>("ping", 50);
    
    _odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    _odom_msg.header.frame_id = _odom_frame;
    _odom_msg.child_frame_id = _base_frame;
    /* [
        1e-3, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1e-3, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1e-3, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1e-3, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1e-3, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1e-3
    ] */
    /* [
         0,  1,  2,  3,  4,  5,
         6,  7,  8,  9, 10, 11,
        12, 13, 14, 15, 16, 17,
        18, 19, 20, 21, 22, 23,
        24, 25, 26, 27, 28, 29,
        30, 31, 32, 33, 34, 35
    ] */
    // odom_msg.pose.covariance.resize(36);
    _odom_msg.pose.covariance[0] = 5e-2;
    _odom_msg.pose.covariance[7] = 5e-2;
    _odom_msg.pose.covariance[14] = 5e-2;
    _odom_msg.pose.covariance[21] = 5e-2;
    _odom_msg.pose.covariance[28] = 5e-2;
    _odom_msg.pose.covariance[35] = 5e-2;

    // odom_msg.twist.covariance.resize(36);
    _odom_msg.twist.covariance[0] = 10e-2;
    _odom_msg.twist.covariance[7] = 10e-2;
    _odom_msg.twist.covariance[14] = 10e-2;
    _odom_msg.twist.covariance[21] = 10e-2;
    _odom_msg.twist.covariance[28] = 10e-2;
    _odom_msg.twist.covariance[35] = 10e-2;

    _twist_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 50, &TJ2NetworkTables::twist_callback, this);
    _prev_twist_timestamp = get_local_time();
    _twist_cmd_speed = 0.0;
    _twist_cmd_dir = 0.0;
    _twist_cmd_vt = 0.0;


    _imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 50);
    _imu_msg.header.frame_id = _imu_frame;
    /* [
        0, 1, 2,
        3, 4, 5,
        6, 7, 8
    ] */
    _imu_msg.orientation_covariance[0] = 10e-5;
    _imu_msg.orientation_covariance[4] = 10e-5;
    _imu_msg.orientation_covariance[8] = 10e-5;

    _imu_msg.angular_velocity_covariance[0] = 10e-5;
    _imu_msg.angular_velocity_covariance[4] = 10e-5;
    _imu_msg.angular_velocity_covariance[8] = 10e-5;
    
    _imu_msg.linear_acceleration_covariance[0] = 100e-5;
    _imu_msg.linear_acceleration_covariance[4] = 100e-5;
    _imu_msg.linear_acceleration_covariance[8] = 100e-5;

    _match_time_pub = nh.advertise<std_msgs::Float64>("match_time", 5);
    _is_autonomous_pub = nh.advertise<std_msgs::Bool>("is_autonomous", 5);

    _odom_reset_srv = nh.advertiseService("odom_reset_service", &TJ2NetworkTables::odom_reset_callback, this);

    ROS_INFO("tj2_networktables init complete");
}

// -----
// Ping methods
// -----

void TJ2NetworkTables::publish_heartbeat()
{
    nt::SetEntryValue(_heartbeat_time_entry, nt::Value::MakeDouble(get_local_time_as_remote()));
    nt::SetEntryValue(_heartbeat_ping_entry, nt::Value::MakeDouble(get_local_time()));
    get_ping_value();
}

void ping_callback(void* data, const NT_EntryNotification* event)
{
    auto ping_value = event->value;
    if (ping_value.type != NT_DOUBLE) {
        ROS_WARN("ping time entry is not a double");
        return;
    }
    ((TJ2NetworkTables*)data)->publish_ping(ping_value.data.v_double);
}

void TJ2NetworkTables::publish_ping(double ping_time)
{
    if (ping_time == 0.0) {
        return;
    }
    std_msgs::Float64 msg;
    msg.data = get_local_time() - ping_time;
    _ping_pub.publish(msg);   
}

void TJ2NetworkTables::get_ping_value()
{
    auto ping_val = nt::GetEntryValue(_return_ping_entry);
    if (!ping_val) {
        ROS_WARN_THROTTLE(5, "ping return value doesn't exist");
        return;
    }
    if (!ping_val->IsDouble()) {
        ROS_WARN_THROTTLE(5, "ping return value is not a double");
        return;
    }
    double ping = ping_val->GetDouble();
    publish_ping(ping);
}


// -----
// Odom
// -----

void TJ2NetworkTables::publish_odom()
{
    auto x_val = nt::GetEntryValue(_x_entry);
    auto y_val = nt::GetEntryValue(_y_entry);
    auto theta_val = nt::GetEntryValue(_theta_entry);
    auto vx_val = nt::GetEntryValue(_vx_entry);
    auto vy_val = nt::GetEntryValue(_vy_entry);
    auto vt_val = nt::GetEntryValue(_vt_entry);

    if (!(x_val && y_val && theta_val && vx_val && vy_val && vt_val)) {
        ROS_WARN_THROTTLE(5, "one of the odom values doesn't exist");
        return;
    }
    if (!(x_val->IsDouble() && y_val->IsDouble() && theta_val->IsDouble() && vx_val->IsDouble() && vy_val->IsDouble() && vt_val->IsDouble())) {
        ROS_WARN_THROTTLE(5, "one of the odom values is not a double");
        return;
    }

    double x = x_val->GetDouble();
    double y = y_val->GetDouble();
    double theta = theta_val->GetDouble();
    double vx = vx_val->GetDouble();
    double vy = vy_val->GetDouble();
    double vt = vt_val->GetDouble();
    
    x *= _remote_linear_units_conversion;
    y *= _remote_linear_units_conversion;
    theta *= _remote_angular_units_conversion;
    vx *= _remote_linear_units_conversion;
    vy *= _remote_linear_units_conversion;
    vt *= _remote_angular_units_conversion;

    tf2::Quaternion quat;
    quat.setRPY(0, 0, theta);

    geometry_msgs::Quaternion msg_quat = tf2::toMsg(quat);

    ros::Time now = ros::Time::now();
    _odom_msg.header.stamp = now;
    _odom_msg.pose.pose.position.x = x;
    _odom_msg.pose.pose.position.y = y;
    _odom_msg.pose.pose.orientation = msg_quat;

    _odom_msg.twist.twist.linear.x = vx;
    _odom_msg.twist.twist.linear.y = vy;
    _odom_msg.twist.twist.angular.z = vt;

    if (_publish_odom_tf)
    {
        geometry_msgs::TransformStamped tf_stamped;
        tf_stamped.header.stamp = now;
        tf_stamped.header.frame_id = _odom_frame;
        tf_stamped.child_frame_id = _base_frame;
        tf_stamped.transform.translation.x = x;
        tf_stamped.transform.translation.y = y;
        tf_stamped.transform.translation.z = 0.0;
        tf_stamped.transform.rotation = msg_quat;

        _tf_broadcaster.sendTransform(tf_stamped);
    }
    
    _odom_pub.publish(_odom_msg);
}


// -----
// Twist
// -----

void TJ2NetworkTables::twist_callback(const geometry_msgs::TwistConstPtr& msg)
{
    double vx = -msg->linear.x;
    double vy = -msg->linear.y;
    double vt = -msg->angular.z;

    if (_zero_epsilon < abs(vx) && abs(vx) < _min_linear_x_cmd) {
        vx = _min_linear_x_cmd;
    }
    if (_zero_epsilon < abs(vy) && abs(vy) < _min_linear_y_cmd) {
        vy = _min_linear_y_cmd;
    }
    if (_zero_epsilon < abs(vt) && abs(vt) < _min_angular_z_cmd) {
        vt = _min_angular_z_cmd;
    }
    
    if (abs(vx) < _zero_epsilon) {
        vx = 0.0;
    }
    if (abs(vy) < _zero_epsilon) {
        vy = 0.0;
    }
    if (abs(vt) < _zero_epsilon) {
        vt = 0.0;
    }
    
    _prev_twist_timestamp = get_local_time();
    _twist_cmd_speed = sqrt(vx * vx + vy * vy) / _remote_linear_units_conversion;
    _twist_cmd_dir = fmod(atan2(vy, vx), 2 * M_PI) / _remote_angular_units_conversion;
    _twist_cmd_vt = vt / _remote_angular_units_conversion;
}


void TJ2NetworkTables::publish_cmd_vel(bool ignore_timeout)
{
    double dt = get_local_time() - _prev_twist_timestamp;
    if (!ignore_timeout && dt > _cmd_vel_timeout) {
        return;
    }
    double remote_time = get_local_time_as_remote();
    
    nt::SetEntryValue(_cmd_time_entry, nt::Value::MakeDouble(remote_time));
    nt::SetEntryValue(_cmd_speed_entry, nt::Value::MakeDouble(_twist_cmd_speed));
    nt::SetEntryValue(_cmd_dir_entry, nt::Value::MakeDouble(_twist_cmd_dir));
    nt::SetEntryValue(_cmd_rotate_entry, nt::Value::MakeDouble(_twist_cmd_vt));
}

// -----
// Imu
// -----

void TJ2NetworkTables::publish_imu()
{
    double timestamp = get_remote_time_as_local();
    if (_prev_imu_timestamp == timestamp) {
        return;
    }
    _prev_imu_timestamp = timestamp;

    ros::Time ros_time = ros::Time::now();

    _imu_msg.header.stamp = ros_time;

    auto yaw_val = nt::GetEntryValue(_imu_yaw_entry);
    if (yaw_val && yaw_val->IsDouble())
    {
        double yaw = yaw_val->GetDouble();

        tf2::Quaternion quat;
        quat.setRPY(0, 0, yaw);

        _imu_msg.orientation = tf2::toMsg(quat);
    }

    auto vz_val = nt::GetEntryValue(_imu_vz_entry);
    if (vz_val && vz_val->IsDouble())
    {
        double vz = vz_val->GetDouble();
        _imu_msg.angular_velocity.z = vz;
    }

    auto ax_val = nt::GetEntryValue(_imu_ax_entry);
    if (ax_val && ax_val->IsDouble())
    {
        double ax = ax_val->GetDouble();
        _imu_msg.linear_acceleration.x = ax;
    }
    auto ay_val = nt::GetEntryValue(_imu_ay_entry);
    if (ay_val && ax_val->IsDouble())
    {
        double ay = ay_val->GetDouble();
        _imu_msg.linear_acceleration.y = ay;
    }

    _imu_pub.publish(_imu_msg);
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
    publish_heartbeat();
    publish_odom();
    publish_cmd_vel();
    publish_imu();
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
