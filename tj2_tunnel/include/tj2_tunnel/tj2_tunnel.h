#pragma once


#include <stdio.h>

#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <fcntl.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <string.h>
#include <mutex>
#include <boost/thread/thread.hpp>

#include "ros/ros.h"
#include "ros/console.h"

#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"

#include "tunnel_protocol.h"

using namespace std;


class TJ2Tunnel {
private:
    ros::NodeHandle nh;  // ROS node handle

    // Parameters
    string _host;
    int _port;
    double _remote_linear_units_conversion;
    double _remote_angular_units_conversion;
    bool _publish_odom_tf;
    string _base_frame;
    string _odom_frame;
    double _cmd_vel_timeout_param;
    ros::Duration _cmd_vel_timeout;
    double _min_linear_x_cmd;
    double _min_linear_y_cmd;
    double _min_angular_z_cmd;
    double _zero_epsilon;
    double _socket_open_attempts;

    // Members
    const int READ_BUFFER_LEN = 4096;
    char* _read_buffer;

    struct sockaddr_in _serv_addr;
    bool _socket_initialized;
    int _socket_id;

    fd_set _socket_set;
    struct timeval _socket_timeout;
    
    int _unparsed_index;
    char* _write_buffer;

    std::mutex _write_lock;

    boost::thread* _poll_socket_thread;

    XmlRpc::XmlRpcValue _categories_param;
    std::map<string, string> _categories;
    TunnelProtocol* protocol;

    ros::Time _last_read_time;
    ros::Duration _last_read_threshold;

    ros::Timer _ping_timer;
    ros::Time _prev_ping_time;
    ros::Duration _ping_interval;

    nav_msgs::Odometry _odom_msg;

    ros::Time _prev_twist_timestamp;
    double _twist_cmd_speed, _twist_cmd_dir, _twist_cmd_vt;

    // Publishers
    tf2_ros::TransformBroadcaster _tf_broadcaster;
    ros::Publisher _odom_pub;
    ros::Publisher _ping_pub;

    // Subscribers
    ros::Subscriber _twist_sub;

    bool reOpenSocket();
    bool openSocket();
    void closeSocket();
    bool didSocketTimeout();

    void writePacket(string category, const char *formats, ...);

    void packetCallback(PacketResult* result);
    void pingCallback(const ros::TimerEvent& event);
    double getLocalTime();

    // void publishPing();
    void publishCmdVel();
    void publishOdom(ros::Time recv_time, double x, double y, double t, double vx, double vy, double vt);

    void twistCallback(const geometry_msgs::TwistConstPtr& msg);

    void pollSocketTask();
    bool pollSocket();

    // Main loop methods
    bool loop();

public:
    TJ2Tunnel(ros::NodeHandle* nodehandle);
    int run();
};
