#pragma once
#include "ros/ros.h"
#include "ros/console.h"
#include <image_transport/image_transport.h>

class TJ2Video
{
private:
    ros::NodeHandle nh;  // ROS node handle
    image_transport::ImageTransport _image_transport;

public:
    TJ2Video(ros::NodeHandle* nodehandle);
    int run();
};