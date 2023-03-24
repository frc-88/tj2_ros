#pragma once

#include "ros/ros.h"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

class TJ2Comm
{
private:
    ros::NodeHandle nh;  // ROS node handle
    
    // Parameters

    
    // Members

    
    // Publishers
    tf2_ros::TransformBroadcaster _tf_broadcaster;

    
    // Subscribers
    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener;

    
    // Sub callbacks

    

public:
    TJ2Comm(ros::NodeHandle* nodehandle);
    int run();
};

