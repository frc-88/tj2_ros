#pragma once

#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/camera_publisher.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/video/video.hpp>
#include <boost/thread/thread.hpp>

using namespace std;

class TJ2Limelight
{
public:
    TJ2Limelight(ros::NodeHandle* nodehandle);
    int run();
private:
    ros::NodeHandle nh;  // ROS node handle

    image_transport::ImageTransport _image_transport;
    image_transport::CameraPublisher _camera_pub;
    string _video_url;
    string _camera_info_url;
    string _frame_id;
    double _max_frame_rate;
    cv::VideoCapture _video_capture;
    camera_info_manager::CameraInfoManager _camera_info_manager;
    boost::thread* _watcher_thread;
    ros::Time _last_publish_time;

    cv_bridge::CvImage _out_msg;
    sensor_msgs::CameraInfo _camera_info;
    sensor_msgs::Image _ros_img;
    ros::Duration _reopenSleep;

    void watchVideoCapture();
    void reopenCapture();
};
