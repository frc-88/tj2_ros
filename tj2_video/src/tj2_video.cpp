
#include "tj2_video.h"

TJ2Video::TJ2Video(ros::NodeHandle* nodehandle) :
    nh(*nodehandle),
    _image_transport(nh)
{
    ros::param::param<int>("~video_width", _width, 640);
    ros::param::param<int>("~video_height", _height, 480);
    ros::param::param<int>("~video_fps", _fps, 30);
    ros::param::param<std::string>("~ffmpeg_command", _ffmpeg_command, "/usr/bin/ffmpeg -y -f rawvideo -vcodec rawvideo -s :width:x:height: -pix_fmt rgb24 -r :fps: -i - -maxrate 2048k -bufsize 256k -flush_packets 1 :path:");
    ros::param::param<std::string>("~output_path", _output_path, "output.mp4");
    ros::param::param<double>("~max_depth", _max_depth, 1.0);
    ros::param::param<bool>("~autostart", _autostart, true);
    _depth = 3;
    _is_open = false;
    _video_duration = ros::Duration(0);
    _current_path = "";

    if (_autostart) {
        if (!open_video(format_path(_output_path))) {
            exit(1);
        }
    }

    _frame_count = 0;
    _image_size = _width * _height * _depth;
    _start_video_srv = nh.advertiseService("start_video", &TJ2Video::start_video_callback, this);
    _stop_video_srv = nh.advertiseService("stop_video", &TJ2Video::stop_video_callback, this);
    _recorded_frame_pub = nh.advertise<std_msgs::Header>("recorded_frame", 10);
    _image_sub = _image_transport.subscribe("image", 10, &TJ2Video::image_callback, this);
}

void TJ2Video::image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    if (!_is_open) {
        return;
    }
    try
    {
        cv::Mat image;
        if (sensor_msgs::image_encodings::isColor(msg->encoding)) {
            image = cv_bridge::toCvShare(msg, "rgb8")->image;
        }
        else {
            image = cv_bridge::toCvShare(msg, msg->encoding)->image;
            int num_channels = sensor_msgs::image_encodings::numChannels(msg->encoding);
            int image_depth = image.type() & CV_MAT_DEPTH_MASK;
            double max_value;
            switch (image_depth)
            {
            case CV_32F:
            case CV_64F:
                max_value = _max_depth;
                break;
            case CV_16U:
                max_value = 0xffff;
                break;
            case CV_16S:
                max_value = 0x7fff;
                break;
            case CV_32S:
                max_value = 0x7ffffff;
                break;
            default:
                max_value = 0xff;
                break;
            }
            image.convertTo(image, CV_32F);
            image *= 255.0 / max_value;
            image.convertTo(image, CV_8U);
            if (num_channels == 1) {
                cv::cvtColor(image, image, CV_GRAY2RGB);
            }
        }
        if (image.cols != _width || image.rows != _height) {
            cv::resize(image, image, cv::Size(_width, _height), cv::INTER_LINEAR);
        }
        fwrite((char*)image.data, 1, _image_size, pipe);
        _frame_count++;
        std_msgs::Header header;
        header.frame_id = _current_path;
        header.stamp = msg->header.stamp;
        header.seq = _frame_count;
        _recorded_frame_pub.publish(header);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'rgb8'.", msg->encoding.c_str());
    }
}

bool TJ2Video::start_video_callback(tj2_interfaces::StartVideo::Request &req, tj2_interfaces::StartVideo::Response &resp)
{
    std::string video_path;
    if (req.path.length() == 0) {
        video_path = _output_path;
    }
    else {
        video_path = req.path;
    }
    video_path = format_path(video_path);
    _video_duration = req.length;
    ROS_INFO("Starting video");
    if (_video_duration > ros::Duration(0)) {
        ROS_INFO_STREAM("Stopping video in " << _video_duration);
    }

    resp.success = open_video(video_path);
    resp.output_path = video_path;
    return resp.success;
}

bool TJ2Video::stop_video_callback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp)
{
    close_video();
    resp.success = true;
    return true;
}

std::string TJ2Video::format_path(std::string path)
{
    auto time = std::time(nullptr);
    auto localtime = *std::localtime(&time);

    std::ostringstream oss;
    oss << std::put_time(&localtime, path.c_str());
    return oss.str();
}

bool TJ2Video::open_video(std::string path)
{
    std::string formatted_command = _ffmpeg_command;
    formatted_command = std::regex_replace(formatted_command, std::regex(":width:"), std::to_string(_width));
    formatted_command = std::regex_replace(formatted_command, std::regex(":height:"), std::to_string(_height));
    formatted_command = std::regex_replace(formatted_command, std::regex(":fps:"), std::to_string(_fps));
    formatted_command = std::regex_replace(formatted_command, std::regex(":path:"), path);
    ROS_INFO_STREAM("command: " << formatted_command);

    if ( !(pipe = popen(formatted_command.c_str(), "w")) ) {
        ROS_ERROR("Failed to start ffmpeg");
        return false;
    }
    ROS_INFO_STREAM("Opening video: " << path);
    _current_path = path;
    _video_start_time = ros::Time::now();
    _is_open = true;
    return true;
}

void TJ2Video::close_video()
{
    if (!_is_open) {
        return;
    }
    ROS_INFO("Closing video");
    _video_duration = ros::Duration(0);
    _is_open = false;
    fflush(pipe);
    fclose(pipe);
}

int TJ2Video::run()
{
    ros::Rate clock_rate(10.0);  // Hz
    while (ros::ok()) {
        ros::spinOnce();
        clock_rate.sleep();
        if (_is_open && 
            _video_duration > ros::Duration(0) &&
            ros::Time::now() - _video_start_time > _video_duration) {
            ROS_INFO("Requested duration exceeded. Closing video.");
            close_video();
        }
    }
    close_video();
    return 0;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "tj2_video");
    ros::NodeHandle nh;

    TJ2Video node(&nh);
    int err = node.run();

    return err;
}
