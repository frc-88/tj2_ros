#include "tj2_tunnel/tj2_tunnel.h"

TJ2Tunnel::TJ2Tunnel(ros::NodeHandle* nodehandle) :
    nh(*nodehandle)
{
    ros::param::param<string>("~host", _host, "127.0.0.1");
    ros::param::param<int>("~port", _port, 3000);

    ros::param::param<double>("~remote_linear_units_conversion", _remote_linear_units_conversion, 0.3048);
    ros::param::param<double>("~remote_angular_units_conversion", _remote_angular_units_conversion, M_PI / 180.0);

    ros::param::param<bool>("~publish_odom_tf", _publish_odom_tf, true);
    ros::param::param<string>("~base_frame", _base_frame, "base_link");
    ros::param::param<string>("~odom_frame", _odom_frame, "odom");

    ros::param::param<double>("~cmd_vel_timeout", _cmd_vel_timeout_param, 0.5);
    ros::param::param<double>("~min_linear_x_cmd", _min_linear_x_cmd, 0.05);
    ros::param::param<double>("~min_linear_y_cmd", _min_linear_y_cmd, 0.05);
    ros::param::param<double>("~min_angular_z_cmd", _min_angular_z_cmd, 0.1);
    ros::param::param<double>("~zero_epsilon", _zero_epsilon, 0.001);

    _cmd_vel_timeout = ros::Duration(_cmd_vel_timeout_param);

    string key;
    if (!ros::param::search("categories", key)) {
        ROS_ERROR("Failed to find categories parameter");
        return;
    }
    nh.getParam(key, _categories_param);

    // _categories_param is a map
    if (_categories_param.getType() != XmlRpc::XmlRpcValue::Type::TypeStruct ||
        _categories_param.size() == 0) {
        ROS_ERROR("categories wrong type or size");
        return;
    }
    
    for (XmlRpc::XmlRpcValue::iterator it = _categories_param.begin(); it != _categories_param.end(); ++it)
    {
        if (it->second.getType() != XmlRpc::XmlRpcValue::TypeString) {
            ROS_WARN("%s category format isn't a string", it->first.c_str());
            continue;
        }
        string category = it->first;
        string format = it->second;
        _categories[category] = format;
    }

    _write_buffer = new char[TunnelProtocol::MAX_PACKET_LEN];
    _read_buffer = new char[READ_BUFFER_LEN];
    _socket_initialized = false;
    _socket_id = 0;

    protocol = new TunnelProtocol(_categories);

    _socket_timeout.tv_sec = 0;
    _socket_timeout.tv_usec = 10000;
    
    if (!openSocket()) {
        return;
    }

    _unparsed_index = 0;

    _prev_ping_time = ros::Time(0);
    _ping_interval = ros::Duration(1.0);

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

    _twist_sub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 50, &TJ2Tunnel::twistCallback, this);
    _prev_twist_timestamp = ros::Time(0);
    _twist_cmd_speed = 0.0;
    _twist_cmd_dir = 0.0;
    _twist_cmd_vt = 0.0;

    _ping_timer = nh.createTimer(ros::Duration(0.5), &TJ2Tunnel::pingCallback, this);

    _poll_socket_thread = new boost::thread(&TJ2Tunnel::pollSocketTask, this);

    ROS_INFO("tj2_tunnel init complete");
}

bool TJ2Tunnel::openSocket()
{
    if ((_socket_id = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        ROS_ERROR("Socket creation error! Failed to create connection.");
        return false;
    }

    _serv_addr.sin_family = AF_INET;
    _serv_addr.sin_port = htons(_port);

    // Convert IPv4 and IPv6 addresses from text to binary form
    if (inet_pton(AF_INET, _host.c_str(), &_serv_addr.sin_addr) <= 0)
    {
        ROS_ERROR("Socket creation error. Invalid server address.");
        return false;
    }

    if (connect(_socket_id, (struct sockaddr *)&_serv_addr, sizeof(_serv_addr)) < 0)
    {
        ROS_ERROR("Socket connection failed!");
        return false;
    }
    _socket_initialized = true;

    return true;
}


void TJ2Tunnel::packetCallback(PacketResult* result)
{
    string category = result->getCategory();
    if (category.compare("odom") == 0) {
        publishOdom(
            result->getRecvTime(),
            result->get_double(0),
            result->get_double(1),
            result->get_double(2),
            result->get_double(3),
            result->get_double(4),
            result->get_double(5)
        );
    }
    else if (category.compare("ping") == 0) {
        std_msgs::Float64 msg;
        double ping_time = result->get_double(0);
        double dt = getLocalTime() - ping_time;
        ROS_DEBUG("Publishing ping time: %f. (Return time: %f)", dt, ping_time);
        msg.data = dt;
        _ping_pub.publish(msg);
    }
}

double TJ2Tunnel::getLocalTime() {
    return ros::Time::now().toSec();
}

void TJ2Tunnel::publishOdom(ros::Time recv_time, double x, double y, double t, double vx, double vy, double vt)
{
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


void TJ2Tunnel::twistCallback(const geometry_msgs::TwistConstPtr& msg)
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
    
    _prev_twist_timestamp = ros::Time::now();
    _twist_cmd_speed = sqrt(vx * vx + vy * vy) / _remote_linear_units_conversion;
    _twist_cmd_dir = fmod(atan2(vy, vx), 2 * M_PI) / _remote_angular_units_conversion;
    _twist_cmd_vt = vt / _remote_angular_units_conversion;
}

void TJ2Tunnel::publishCmdVel()
{
    ros::Duration dt = ros::Time::now() - _prev_twist_timestamp;
    if (dt > _cmd_vel_timeout) {
        // ROS_DEBUG_THROTTLE(2.0, "cmd_vel timed out skipping write.");
        return;
    }

    writePacket("cmd", "fff", _twist_cmd_dir, _twist_cmd_speed, _twist_cmd_vt);
}


void TJ2Tunnel::pingCallback(const ros::TimerEvent& event)
{
    writePacket("ping", "f", getLocalTime());
}

// void TJ2Tunnel::publishPing()
// {
//     ros::Time now = ros::Time::now();
//     if (now - _prev_ping_time > _ping_interval) {
//         writePacket("ping", "f", getLocalTime());
//         _prev_ping_time = now;
//     }
// }


void TJ2Tunnel::writePacket(string category, const char *formats, ...)
{
    va_list args;
    va_start(args, formats);
    _write_lock.lock();
    int length = protocol->makePacket(_write_buffer, category, formats, args);
    ROS_DEBUG("Writing packet: %s", protocol->packetToString(_write_buffer, 0, length).c_str());
    if (length > 0) {
        write(_socket_id, _write_buffer, length);
    }
    else {
        ROS_DEBUG("Skipping write for packet: %s. Length is %d", protocol->packetToString(_write_buffer, 0, length).c_str(), length);
    }
    _write_lock.unlock();
    va_end(args);
}


bool TJ2Tunnel::pollSocket()
{
    if (!_socket_initialized) {
        ROS_WARN("Socket is not initialized. Exiting.");
        return false;
    }
    FD_ZERO(&_socket_set);  // clear the set
    FD_SET(_socket_id, &_socket_set); // add our file descriptor to the set
    int return_val = select(_socket_id + 1, &_socket_set, NULL, NULL, &_socket_timeout);
    if (return_val == -1) {
        ROS_ERROR("An error occurred while checking the socket for available data");
        return false;
    }
    else if (return_val == 0) {
        return true;  // a timeout occurred
    }
    int num_chars_read = read(_socket_id, _read_buffer, READ_BUFFER_LEN);
    if (num_chars_read == -1) {
        ROS_WARN("Socket indicated it should exit.");
        return false;
    }
    int last_parsed_index = protocol->parseBuffer(_read_buffer, 0, _unparsed_index + num_chars_read);

    PacketResult* result;
    do {
        result = protocol->popResult();
        if (result->getErrorCode() == TunnelProtocol::NULL_ERROR) {
            continue;
        }
        if (protocol->isCodeError(result->getErrorCode())) {
            ROS_ERROR("Encountered error code %d.", result->getErrorCode());
            continue;
        }
        packetCallback(result);
    }
    while (result->getErrorCode() != TunnelProtocol::NULL_ERROR);

    _unparsed_index = _unparsed_index + num_chars_read - last_parsed_index;
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

void TJ2Tunnel::pollSocketTask()
{
    while (ros::ok())
    {
        if (!pollSocket()) {
            ROS_INFO("Exiting socket thread");
            break;
        }
    }
    closeSocket();
}

void TJ2Tunnel::closeSocket()
{
    close(_socket_id);
    _socket_initialized = false;
}


bool TJ2Tunnel::loop()
{
    // if (!pollSocket()) {
    //     return false;
    // }
    publishCmdVel();
    // publishPing();
    return true;
}

int TJ2Tunnel::run()
{
    ros::Rate clock_rate(100);  // Hz

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
    _poll_socket_thread->join();
    return exit_code;
}
