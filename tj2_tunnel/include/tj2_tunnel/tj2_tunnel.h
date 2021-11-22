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

#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>

#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/Imu.h"

#include "tj2_tunnel/SwerveModule.h"
#include "tj2_tunnel/SwerveMotor.h"
#include "tj2_tunnel/OdomReset.h"

#include "tunnel_protocol.h"

using namespace std;

int sign_of(double x) {
    return (x > 0) - (x < 0);
}

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
    string _map_frame;
    string _imu_frame;
    double _cmd_vel_timeout_param;
    ros::Duration _cmd_vel_timeout;
    double _min_linear_cmd;
    double _min_angular_z_cmd;
    double _zero_epsilon;
    double _socket_open_attempts;
    int _num_modules;

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

    // Messages
    nav_msgs::Odometry _odom_msg;
    sensor_msgs::Imu _imu_msg;
    vector<tj2_tunnel::SwerveModule*>* _module_msgs;

    ros::Time _prev_twist_timestamp;
    double _twist_cmd_vx, _twist_cmd_vy, _twist_cmd_vt;

    // Publishers
    tf2_ros::TransformBroadcaster _tf_broadcaster;
    ros::Publisher _odom_pub;
    ros::Publisher _ping_pub;
    ros::Publisher _imu_pub;
    vector<ros::Publisher>* _module_pubs;

    // Subscribers
    ros::Subscriber _twist_sub;
    ros::Subscriber _global_plan_sub;

    // Service Servers
    ros::ServiceServer _odom_reset_srv;

    // ROS TF
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    bool reOpenSocket();
    bool openSocket();
    void closeSocket();
    bool didSocketTimeout();

    void writePacket(string category, const char *formats, ...);

    void packetCallback(PacketResult* result);
    void pingCallback(const ros::TimerEvent& event);
    double getLocalTime();

    // Service callbacks
    bool odom_reset_callback(tj2_tunnel::OdomReset::Request &req, tj2_tunnel::OdomReset::Response &resp);

    // void publishPing();
    void publishCmdVel();
    void publishGlobalPose();
    void publishOdom(ros::Time recv_time, double x, double y, double t, double vx, double vy, double vt);
    void publishImu(ros::Time recv_time, double yaw, double yaw_rate, double accel_x, double accel_y);
    void publishModule(ros::Time recv_time,
        int module_index,
        double module_angle, double module_speed,
        double lo_voltage_command, double lo_radps,
        double hi_voltage_command, double hi_radps
    );

    void twistCallback(const geometry_msgs::TwistConstPtr& msg);
    void globalPathCallback(const nav_msgs::PathConstPtr& msg);

    void pollSocketTask();
    bool pollSocket();

    // Main loop methods
    bool loop();

public:
    TJ2Tunnel(ros::NodeHandle* nodehandle);
    int run();
};
