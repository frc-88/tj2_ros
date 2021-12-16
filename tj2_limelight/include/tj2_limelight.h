#pragma once

#include <camera_info_manager/camera_info_manager.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/camera_publisher.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui/highgui.hpp>
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

    // Members
    ros::Duration _reopenSleep;
    camera_info_manager::CameraInfoManager _camera_info_manager;
    image_transport::ImageTransport _image_transport;
    sensor_msgs::CameraInfo _camera_info;

    boost::thread* _watcher_thread;
    ros::Time _last_publish_time;

    // Publishers
    image_transport::CameraPublisher _camera_pub;

    // Parameters
    string _video_url;
    string _camera_info_url;
    string _frame_id;
    double _max_frame_rate;

    // OpenCV video capture
    cv::VideoCapture _video_capture;

    // Messages
    cv_bridge::CvImage _out_msg;
    sensor_msgs::Image _ros_img;

    void watchVideoCapture();
    void reopenCapture();
};
