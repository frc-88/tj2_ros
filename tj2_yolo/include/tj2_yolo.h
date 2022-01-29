#pragma once
#include "ros/ros.h"
#include "ros/console.h"

#include "detector.h"


class TJ2Yolo
{
public:
    TJ2Yolo(ros::NodeHandle* nodehandle);
    int run();
private:
    ros::NodeHandle nh;  // ROS node handle

    // Parameters
    std::string _model_path;
    std::string _classes_path;
    float _conf_threshold;
    float _iou_threshold;
    std::string _image_width_param;
    std::string _image_height_param;
    int _image_width;
    int _image_height;

    // Members
    Detector* detector;
    std::vector<std::string> class_names;
};

