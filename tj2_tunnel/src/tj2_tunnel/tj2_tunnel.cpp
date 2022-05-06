#include "tj2_tunnel/tj2_tunnel.h"

TJ2Tunnel::TJ2Tunnel(ros::NodeHandle* nodehandle) :
    nh(*nodehandle)
{
    ros::param::param<string>("~device_path", _device_path, "/dev/ttyTHS0");
    ros::param::param<int>("~device_baud", _device_baud, 115200);

    ros::param::param<double>("~remote_linear_units_conversion", _remote_linear_units_conversion, 0.3048);
    ros::param::param<double>("~remote_angular_units_conversion", _remote_angular_units_conversion, M_PI / 180.0);

    ros::param::param<double>("~tunnel_rate", _tunnel_rate, 30.0);

    ros::param::param<bool>("~publish_odom_tf", _publish_odom_tf, true);
    ros::param::param<string>("~base_frame", _base_frame, "base_link");
    ros::param::param<string>("~odom_frame", _odom_frame, "odom");
    ros::param::param<string>("~map_frame", _map_frame, "map");
    ros::param::param<string>("~imu_frame", _imu_frame, "imu");

    ros::param::param<double>("~cmd_vel_timeout", _cmd_vel_timeout_param, 0.5);
    ros::param::param<double>("~min_linear_cmd", _min_linear_cmd, 0.05);
    ros::param::param<double>("~min_angular_z_cmd", _min_angular_z_cmd, 0.1);
    ros::param::param<double>("~zero_epsilon", _zero_epsilon, 0.001);

    ros::param::param<double>("~pose_estimate_x_std", _pose_estimate_x_std, 0.5);
    ros::param::param<double>("~pose_estimate_y_std", _pose_estimate_y_std, 0.5);
    ros::param::param<double>("~pose_estimate_theta_std_deg", _pose_estimate_theta_std_deg, 15.0);
    ros::param::param<string>("~pose_estimate_frame_id", _pose_estimate_frame_id, "map");

    ros::param::param<int>("~open_attempts", _open_attempts, 50);

    string key;
    if (!ros::param::search("joint_names", key)) {
        ROS_ERROR("Failed to find joint_names parameter");
        std::exit(EXIT_FAILURE);
    }
    ROS_DEBUG("Found joint_names: %s", key.c_str());
    nh.getParam(key, _joint_names);

    _cmd_vel_timeout = ros::Duration(_cmd_vel_timeout_param);

    _write_buffer = new char[TunnelProtocol::MAX_PACKET_LEN];
    _read_buffer = new char[READ_BUFFER_LEN];
    _initialized = false;

    _protocol = new TunnelProtocol();

    if (!reOpenDevice()) {
        return;
    }

    _unparsed_index = 0;

    _prev_ping_time = ros::Time(0);
    _ping_interval = ros::Duration(1.0);

    _last_read_time = ros::Time(0);
    _last_read_threshold = ros::Duration(5.0);

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

    _raw_joint_pubs = new vector<ros::Publisher>();
    _raw_joint_msgs = new vector<std_msgs::Float64*>();
    
    for (int index = 0; index < _joint_names.size(); index++) {
        addJointPub(_joint_names.at(index));
    }

    _match_time_pub = nh.advertise<std_msgs::Float64>("match_time", 10);
    _autonomous_pub = nh.advertise<std_msgs::Bool>("is_autonomous", 10);
    _team_color_pub = nh.advertise<std_msgs::String>("team_color", 10);

    _pose_estimate_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);

    _packet_count_pub = nh.advertise<std_msgs::Int32>("packet_count", 10);
    _packet_rate_pub = nh.advertise<std_msgs::Float64>("packet_rate", 10);

    _waypoints_action_client = new actionlib::SimpleActionClient<tj2_waypoints::FollowPathAction>("follow_path", true);

    _twist_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 50, &TJ2Tunnel::twistCallback, this);

    _prev_twist_timestamp = ros::Time(0);
    _twist_cmd_vx = 0.0;
    _twist_cmd_vy = 0.0;
    _twist_cmd_vt = 0.0;

    _currentGoalStatus = INVALID;

    _status_prev_time = ros::Time::now();
    _status_prev_count = 0;
    _packet_count = 0;

    _odom_reset_srv = nh.advertiseService("odom_reset_service", &TJ2Tunnel::odom_reset_callback, this);

    _ping_timer = nh.createTimer(ros::Duration(0.5), &TJ2Tunnel::pingCallback, this);

    _poll_thread = new boost::thread(&TJ2Tunnel::pollDeviceTask, this);

    ROS_INFO("tj2_tunnel init complete");
}

void TJ2Tunnel::addJointPub(string name)
{
    ROS_INFO("Subscribing to joint topic: %s", name.c_str());
    _raw_joint_pubs->push_back(nh.advertise<std_msgs::Float64>(name, 50));
    _raw_joint_msgs->push_back(new std_msgs::Float64);
}

bool TJ2Tunnel::reOpenDevice()
{
    for (int attempt = 0; attempt < _open_attempts; attempt++)
    {
        if (!ros::ok()) {
            ROS_INFO("Exiting reopen");
            break;
        }
        ros::Duration(2.0).sleep();
        if (attempt > 0) {
            ROS_INFO("Open device attempt #%d", attempt + 1);
        }
        closeDevice();
        if (openDevice()) {
            break;
        }
        ROS_INFO("Connection attempt failed");
    }
    if (!_initialized) {
        ROS_ERROR("Maximum number of attempts reached");
    }
    return _initialized;
}

bool TJ2Tunnel::openDevice()
{
    ROS_INFO("Initializing device");

    _device.setPort(_device_path);
    _device.setBaudrate(_device_baud);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
    _device.setTimeout(timeout);
    _device.open();

    if (!_device.isOpen()) {
        return false;
    }

    _initialized = true;
    ROS_INFO("Device initialized");

    return true;
}
bool TJ2Tunnel::didDeviceTimeout()
{
    if (ros::Time::now() - _last_read_time > _last_read_threshold) {
        ROS_INFO("Device timed out while waiting for data");
        _last_read_time = ros::Time::now();
        return true;
    }
    else {
        return false;
    }
}


void TJ2Tunnel::packetCallback(PacketResult* result)
{
    _packet_count++;
    string category = result->getCategory();
    if (category.compare("odom") == 0) {
        double x = result->getDouble();
        double y = result->getDouble();
        double t = result->getDouble();
        double vx = result->getDouble();
        double vy = result->getDouble();
        double vt = result->getDouble();
        publishOdom(
            result->getRecvTime(),
            x, y, t, vx, vy, vt
        );
    }
    else if (category.compare("ping") == 0) {
        double ping_time = result->getDouble();
        double dt = getLocalTime() - ping_time;
        ROS_DEBUG("Publishing ping time: %f. (Return time: %f)", dt, ping_time);
        publishStatusMessages(dt);
    }
    else if (category.compare("imu") == 0) {
        double yaw = result->getDouble();
        double yaw_rate = result->getDouble();
        double accel_x = result->getDouble();
        double accel_y = result->getDouble();
        publishImu(
            result->getRecvTime(),
            yaw, yaw_rate, accel_x, accel_y
        );
    }
    else if (category.compare("joint") == 0) {
        /* For some reason, if publishJoint is called like this:
            publishJoint(
                result->getRecvTime(),
                result->getInt(), result->getDouble()
            );
            getDouble gets called before getInt... Separating out into variables fixes this  
        */
        int index = result->getInt();
        double value = result->getDouble();
        publishJoint(
            result->getRecvTime(),
            index, value
        );
    }
    else if (category.compare("goal") == 0 || category.compare("gpose") == 0) {
        string waypoint_name = "";
        geometry_msgs::Pose pose;
        if (category.compare("goal") == 0) {
            waypoint_name = result->getString();
        }
        else if (category.compare("gpose") == 0) {
            double x = result->getDouble();
            double y = result->getDouble();
            double theta = result->getDouble();

            tf2::Quaternion quat;
            quat.setRPY(0, 0, theta);

            geometry_msgs::Quaternion msg_quat = tf2::toMsg(quat);

            pose.position.x = x;
            pose.position.y = y;
            pose.orientation = msg_quat;
        }
        bool is_continuous = result->getInt();
        bool ignore_orientation = result->getInt();
        double intermediate_tolerance = result->getDouble();
        bool ignore_obstacles = result->getInt();
        bool ignore_walls = result->getInt();
        string interruptable_by = result->getString();
        
        tj2_waypoints::Waypoint waypoint;
        waypoint.name = waypoint_name;
        waypoint.pose = pose;
        waypoint.is_continuous = is_continuous;
        waypoint.ignore_orientation = ignore_orientation;
        waypoint.intermediate_tolerance = intermediate_tolerance;
        waypoint.ignore_obstacles = ignore_obstacles;
        waypoint.ignore_walls = ignore_walls;
        waypoint.interruptable_by = interruptable_by;
        _waypoints.waypoints.insert(_waypoints.waypoints.end(), waypoint);
        ROS_INFO("Received a waypoint: %s. pose: x=%0.4f, y=%0.4f. is_continuous: %d, ignore_orientation: %d, ignore_obstacles: %d, ignore_walls: %d, interruptable_by: %s",
            waypoint_name.c_str(),
            pose.position.x,
            pose.position.y,
            is_continuous,
            ignore_orientation,
            ignore_obstacles,
            ignore_walls,
            interruptable_by.c_str()
        );
    }
    else if (category.compare("exec") == 0) {
        ROS_INFO("Received execute plan command");
        int num_waypoints = result->getInt();
        if (num_waypoints != _waypoints.waypoints.size()) {
            ROS_ERROR("The reported number of waypoints in the plan does match the number received! %d != %ld Canceling plan", num_waypoints, _waypoints.waypoints.size());
            setGoalStatus(GoalStatus::FAILED);
        }
        else {
            sendWaypoints();
        }
        resetWaypoints();
    }
    else if (category.compare("reset") == 0) {
        ROS_INFO("Received reset plan command");
        resetWaypoints();
    }
    else if (category.compare("cancel") == 0) {
        ROS_INFO("Received cancel plan command");
        cancelWaypointGoal();
    }
    else if (category.compare("match") == 0) {
        publishMatch(
            (bool)result->getInt(),
            result->getDouble(),
            result->getString()
        );
    }
    else if (category.compare("poseest") == 0) {
        sendPoseEstimate(
            result->getDouble(),
            result->getDouble(),
            result->getDouble()
        );
    }
}

double TJ2Tunnel::getLocalTime() {
    return ros::Time::now().toSec();
}

void TJ2Tunnel::publishStatusMessages(float ping)
{
    std_msgs::Float64 ping_msg;
    ping_msg.data = ping;
    _ping_pub.publish(ping_msg);

    int num_messages = _packet_count - _status_prev_count;
    ros::Duration status_interval = ros::Time::now() - _status_prev_time;
    double rate = (double)num_messages / status_interval.toSec();

    std_msgs::Int32 count_msg;
    count_msg.data = _packet_count;
    _packet_count_pub.publish(count_msg);

    std_msgs::Float64 rate_msg;
    rate_msg.data = rate;
    _packet_rate_pub.publish(rate_msg);

    _status_prev_count = _packet_count;
    _status_prev_time = ros::Time::now();
    
}

void TJ2Tunnel::publishOdom(ros::Time recv_time, double x, double y, double t, double vx, double vy, double vt)
{
    if (!(std::isfinite(x) && std::isfinite(y) && std::isfinite(t) && std::isfinite(vx) && std::isfinite(vy) && std::isfinite(vt))) {
        ROS_WARN_THROTTLE(1.0, "Odometry values are nan or inf");
        return;
    }
    x *= _remote_linear_units_conversion;
    y *= _remote_linear_units_conversion;
    t *= _remote_angular_units_conversion;
    vx *= _remote_linear_units_conversion;
    vy *= _remote_linear_units_conversion;
    vt *= _remote_angular_units_conversion;

    tf2::Quaternion quat;
    quat.setRPY(0, 0, t);

    geometry_msgs::Quaternion msg_quat = tf2::toMsg(quat);

    _odom_msg.header.stamp = recv_time;
    _odom_msg.pose.pose.position.x = x;
    _odom_msg.pose.pose.position.y = y;
    _odom_msg.pose.pose.orientation = msg_quat;

    _odom_msg.twist.twist.linear.x = vx;
    _odom_msg.twist.twist.linear.y = vy;
    _odom_msg.twist.twist.angular.z = vt;

    if (_publish_odom_tf)
    {
        geometry_msgs::TransformStamped tf_stamped;
        tf_stamped.header.stamp = recv_time;
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

void TJ2Tunnel::publishImu(ros::Time recv_time, double yaw, double yaw_rate, double accel_x, double accel_y)
{
    if (!(std::isfinite(yaw) && std::isfinite(yaw_rate) && std::isfinite(accel_x) && std::isfinite(accel_y))) {
        ROS_WARN_THROTTLE(1.0, "A value for the IMU is nan or inf");
    }

    _imu_msg.header.stamp = recv_time;

    yaw *= M_PI / 180.0;
    yaw_rate *= M_PI / 180.0;
    accel_x *= 9.81;
    accel_y *= 9.81;

    tf2::Quaternion quat;
    quat.setRPY(0, 0, yaw);
    _imu_msg.orientation = tf2::toMsg(quat);
    _imu_msg.angular_velocity.z = yaw_rate;
    _imu_pub.publish(_imu_msg);
}

void TJ2Tunnel::publishJoint(ros::Time recv_time, int joint_index, double joint_position)
{
    if (joint_index < 0 || joint_index >= _raw_joint_msgs->size()) {
        ROS_WARN("Invalid joint index received: %d. Valid range is 0..%lu. (Joint value was %f. recv time is %f)", joint_index, _raw_joint_msgs->size() - 1, joint_position, recv_time.toSec());
        return;
    }
    if (!std::isfinite(joint_position)) {
        ROS_WARN_THROTTLE(1.0, "Joint position for index %d is nan or inf", joint_index);
        return;
    }
    std_msgs::Float64* msg = _raw_joint_msgs->at(joint_index);
    msg->data = joint_position;

    _raw_joint_pubs->at(joint_index).publish(*msg);
}

void TJ2Tunnel::publishGoalStatus()
{
    actionlib::SimpleClientGoalState state = _waypoints_action_client->getState();

    GoalStatus currentPollStatus;

    // Possible states:  PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST
    if (state.isDone()) {
        // RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, or LOST.
        if (state == actionlib::SimpleClientGoalState::StateEnum::SUCCEEDED) {
            currentPollStatus = GoalStatus::FINISHED;
        }
        else {
            currentPollStatus = GoalStatus::FAILED;
        }
    }
    else {
        // PENDING or ACTIVE
        currentPollStatus = GoalStatus::RUNNING;
    }

    if (currentPollStatus != _prevPollStatus) {
        ROS_INFO("Current goal status changed to: %d", _currentGoalStatus);
        _prevPollStatus = currentPollStatus;
        _currentGoalStatus = currentPollStatus;
    }
    writePacket("gstatus", "d", _currentGoalStatus);
}

void TJ2Tunnel::publishRobotGlobalPose()
{
    tf::StampedTransform transform;
    try {
        _tf_listener.lookupTransform(_base_frame, _map_frame, ros::Time(0), transform);
    }
    catch (tf::TransformException ex) {
        return;
    }

    double x = transform.getOrigin().x();
    double y = transform.getOrigin().y();
    double theta = tf::getYaw(transform.getRotation()); 
    writePacket("global", "fff", x, y, theta);
}

void TJ2Tunnel::setGoalStatus(GoalStatus status)
{
    _currentGoalStatus = status;
}

void TJ2Tunnel::sendWaypoints()
{
    ROS_INFO("Sending waypoints");
    tj2_waypoints::FollowPathGoal goal;
    goal.waypoints = _waypoints;
    _waypoints_action_client->sendGoal(goal);
}

void TJ2Tunnel::resetWaypoints() {
    _waypoints.waypoints.clear();
}

void TJ2Tunnel::cancelWaypointGoal()
{
    ROS_INFO("Canceling waypoint goal");
    _waypoints_action_client->cancelAllGoals();
    resetWaypoints();
}

void TJ2Tunnel::publishMatch(bool is_autonomous, double match_timer, string team_color)
{
    std_msgs::Float64 timer_msg;
    timer_msg.data = match_timer;
    _match_time_pub.publish(timer_msg);

    std_msgs::Bool is_auto_msg;
    is_auto_msg.data = is_autonomous;
    _autonomous_pub.publish(is_auto_msg);

    std_msgs::String team_color_msg;
    team_color_msg.data = team_color;
    _team_color_pub.publish(team_color_msg);
}

void TJ2Tunnel::sendPoseEstimate(double x, double y, double theta)
{
    geometry_msgs::PoseWithCovarianceStamped pose_est;

    /*
    [
        x_std * x_std, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, y_std * y_std, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, theta_std_rad * theta_std_rad
    ]
    [
         0,  1,  2,  3,  4,  5,
         6,  7,  8,  9, 10, 11,
        12, 13, 14, 15, 16, 17,
        18, 19, 20, 21, 22, 23,
        24, 25, 26, 27, 28, 29,
        30, 31, 32, 33, 34, 35
    ]
    */

    tf2::Quaternion quat;
    quat.setRPY(0.0, 0.0, theta);

    geometry_msgs::Quaternion msg_quat = tf2::toMsg(quat);

    pose_est.pose.pose.position.x = x;
    pose_est.pose.pose.position.y = y;
    pose_est.pose.pose.orientation = msg_quat;
    pose_est.header.frame_id = _pose_estimate_frame_id;

    double theta_std_rad = _pose_estimate_theta_std_deg * M_PI / 180.0;

    pose_est.pose.covariance[0] = _pose_estimate_x_std * _pose_estimate_x_std;
    pose_est.pose.covariance[7] = _pose_estimate_y_std * _pose_estimate_y_std;
    pose_est.pose.covariance[35] = theta_std_rad * theta_std_rad;

    _pose_estimate_pub.publish(pose_est);
}

void TJ2Tunnel::twistCallback(const geometry_msgs::TwistConstPtr& msg)
{
    double vx = msg->linear.x;
    double vy = msg->linear.y;
    double vt = msg->angular.z;
    
    // If magnitude of translation is in the "no-go" zone (_zero_epsilon..._min_linear_cmd),
    // set vx, vy to _min_linear_cmd with heading applied
    double trans_vel = sqrt(vx * vx + vy * vy);
    if (_zero_epsilon < abs(trans_vel) && abs(trans_vel) < _min_linear_cmd)
    {
        double trans_angle = atan2(vy, vx);
        vx = _min_linear_cmd * cos(trans_angle);
        vy = _min_linear_cmd * sin(trans_angle);
    }
    // If magnitude of translation is in the "zero" zone (<_zero_epsilon),
    // Set translation velocity to zero
    //      If angular velocity is in the "no-go" zone,
    //      set vt to _min_angular_z_cmd with direction applied
    //      If angular velocity is in the "zero" zone,
    //      set vt to zero
    else if (abs(trans_vel) < _zero_epsilon) {
        vx = 0.0;
        vy = 0.0;
        if (_zero_epsilon < abs(vt) && abs(vt) < _min_angular_z_cmd) {
            vt = sign_of(vt) * _min_angular_z_cmd;
        }
        else if (abs(vt) < _zero_epsilon) {
            vt = 0.0;
        }
    }

    _prev_twist_timestamp = ros::Time::now();
    _twist_cmd_vx = vx / _remote_linear_units_conversion;
    _twist_cmd_vy = vy / _remote_linear_units_conversion;
    _twist_cmd_vt = vt / _remote_angular_units_conversion;
}

void TJ2Tunnel::publishCmdVel()
{
    ros::Duration dt = ros::Time::now() - _prev_twist_timestamp;
    if (dt > _cmd_vel_timeout) {
        ROS_DEBUG_THROTTLE(5.0, "cmd_vel timed out skipping write.");
        return;
    }

    writePacket("cmd", "fff", _twist_cmd_vx, _twist_cmd_vy, _twist_cmd_vt);
}


void TJ2Tunnel::pingCallback(const ros::TimerEvent& event) {
    double ping_time = getLocalTime();
    ROS_DEBUG("Writing ping time: %f", ping_time);
    writePacket("ping", "f", ping_time);
}


void TJ2Tunnel::writePacket(string category, const char *formats, ...)
{
    if (!_initialized) {
        ROS_DEBUG("Device is not initialized. Skipping write. Category: %s", category.c_str());
        return;
    }
    va_list args;
    va_start(args, formats);
    _write_lock.lock();
    int length = _protocol->makePacket(_write_buffer, category, formats, args);
    ROS_DEBUG("Writing packet: %s", packetToString(_write_buffer, 0, length).c_str());
    if (length > 0) {
        _device.write((uint8_t*)_write_buffer, length);
    }
    else {
        ROS_DEBUG("Skipping write for packet: %s. Length is %d", packetToString(_write_buffer, 0, length).c_str(), length);
    }
    _write_lock.unlock();
    va_end(args);
}


bool TJ2Tunnel::pollDevice()
{
    if (!_initialized) {
        ROS_WARN("Device is not initialized.");
        reOpenDevice();
        return true;
    }

    int num_chars_read = _device.available();
    if (num_chars_read <= 0) {
        if (didDeviceTimeout()) {
            reOpenDevice();
        }
        return true;
    }
    _device.read((uint8_t*)(_read_buffer + _unparsed_index), READ_BUFFER_LEN - _unparsed_index);

    _last_read_time = ros::Time::now();
    int read_stop_index = _unparsed_index + num_chars_read;
    // ROS_INFO("_unparsed_index: %d, num_chars_read: %d", _unparsed_index, num_chars_read);
    int last_parsed_index = _protocol->parseBuffer(_read_buffer, 0, read_stop_index);

    PacketResult* result;
    do {
        result = _protocol->popResult();
        if (result->getErrorCode() == TunnelProtocol::NULL_ERROR) {
            continue;
        }
        if (_protocol->isCodeError(result->getErrorCode())) {
            ROS_ERROR("Encountered error code %d.", result->getErrorCode());
            continue;
        }
        string category = result->getCategory();
        if (category.compare("__msg__") == 0) {
            ROS_INFO("Tunnel message: %s", result->getString().c_str());
        }
        else {
            packetCallback(result);
        }
    }
    while (result->getErrorCode() != TunnelProtocol::NULL_ERROR);

    _unparsed_index = read_stop_index - last_parsed_index;
    if (_unparsed_index >= READ_BUFFER_LEN) {
        _unparsed_index = 0;
    }

    if (last_parsed_index > 0) {
        for (int index = last_parsed_index, shifted_index = 0; index < READ_BUFFER_LEN; index++, shifted_index++) {
            _read_buffer[shifted_index] = _read_buffer[index];
        }
    }

    return true;
}

void TJ2Tunnel::pollDeviceTask()
{ 
    ros::Rate clock_rate(200);  // Hz

    while (ros::ok())
    {
        if (!pollDevice()) {
            ROS_INFO("Exiting device poll thread");
            break;
        }
        clock_rate.sleep();
    }
    closeDevice();
}

void TJ2Tunnel::closeDevice()
{
    _device.close();
    _initialized = false;
}

bool TJ2Tunnel::odom_reset_callback(tj2_tunnel::OdomReset::Request &req, tj2_tunnel::OdomReset::Response &resp)
{
    writePacket("reset", "fff", req.x, req.y, req.t);
    ROS_INFO("Resetting odometry to x: %0.3f, y: %0.3f, theta: %0.3f", req.x, req.y, req.t);
    resp.resp = true;
    return true;
}

bool TJ2Tunnel::loop()
{
    publishCmdVel();
    publishGoalStatus();
    publishRobotGlobalPose();
    return true;
}

int TJ2Tunnel::run()
{
    ros::Rate clock_rate(_tunnel_rate);  // Hz

    int exit_code = 0;
    while (ros::ok())
    {
        // let ROS process any events
        ros::spinOnce();
        clock_rate.sleep();

        try {
            if (!loop()) {
                break;
            }
        }
        catch (exception& e) {
            ROS_ERROR_STREAM("Exception in main loop: " << e.what());
            exit_code = 1;
            break;
        }
    }
    _poll_thread->join();
    return exit_code;
}
