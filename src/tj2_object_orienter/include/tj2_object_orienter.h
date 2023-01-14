#pragma once

#include "ros/ros.h"
#include "ros/console.h"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <sensor_msgs/PointCloud2.h>

#include "open3d/Open3D.h"

class TJ2ObjectOrienter {
public:
    TJ2ObjectOrienter(ros::NodeHandle* nodehandle);
    int run();

private:
    ros::NodeHandle nh;  // ROS node handle
    ros::Subscriber _point_cloud_sub;

    // Message callbacks
    void cloud_callback(const sensor_msgs::PointCloud2ConstPtr& msg);

    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener;
};
