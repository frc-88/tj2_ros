#pragma once

#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>

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

    // Members
    const int READ_BUFFER_LEN = 1024;
    char* _read_buffer;

    struct sockaddr_in _serv_addr;
    bool _socket_initialized;
    int _socket_id;
    
    int _unparsed_index;

    XmlRpc::XmlRpcValue _categories_param;
    std::map<string, string> _categories;
    TunnelProtocol* protocol;

    // Publishers
    tf2_ros::TransformBroadcaster _tf_broadcaster;

    void packetCallback(PacketResult* result);

    // Main loop methods
    bool loop();

public:
    TJ2Tunnel(ros::NodeHandle* nodehandle);
    int run();
};
