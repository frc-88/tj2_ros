#pragma once

#include <iostream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <regex>

#include "ros/ros.h"
#include "ros/console.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <std_msgs/Header.h>
#include <tj2_interfaces/StartVideo.h>
#include <std_srvs/Trigger.h>


class TJ2Video
{
private:
    ros::NodeHandle nh;  // ROS node handle
    image_transport::ImageTransport _image_transport;

    FILE *pipe;
    long _image_size;
    int _fps;
    int _width, _height, _depth;
    uint32_t _frame_count;
    std::string _ffmpeg_command;
    std::string _output_path;
    std::string _current_path;
    bool _is_open;
    bool _autostart;
    double _max_depth;
    ros::Time _video_start_time;
    ros::Duration _video_duration;

    image_transport::Subscriber _image_sub;
    ros::Publisher _recorded_frame_pub;
    ros::ServiceServer _start_video_srv;
    ros::ServiceServer _stop_video_srv;

    bool start_video_callback(tj2_interfaces::StartVideo::Request &req, tj2_interfaces::StartVideo::Response &resp);
    bool stop_video_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp);

    std::string format_path(std::string path);
    void image_callback(const sensor_msgs::ImageConstPtr& msg);
    bool open_video(std::string path);
    void close_video();

public:
    TJ2Video(ros::NodeHandle* nodehandle);
    int run();
};