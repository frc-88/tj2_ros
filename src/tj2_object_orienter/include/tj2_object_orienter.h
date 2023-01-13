#pragma once

#include "ros/ros.h"
#include "ros/console.h"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>


class TJ2ObjectOrienter {
public:
    TJ2ObjectOrienter(ros::NodeHandle* nodehandle);
    int run();

private:
    ros::NodeHandle nh;  // ROS node handle

    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener;
};
