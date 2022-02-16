#pragma once

#include "ntcore.h"

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

#include <tj2_limelight/LimelightTargetArray.h>

#include <std_msgs/Bool.h>

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
    sensor_msgs::CameraInfo _camera_info;
    image_transport::ImageTransport _image_transport;

    boost::thread* _watcher_thread;
    ros::Time _last_publish_time;

    NT_Inst _nt;
    string _nt_host;
    int _nt_port;
    int _num_limelight_targets;

    NT_Entry _limelight_led_mode_entry;
    NT_Entry _limelight_cam_mode_entry;
    NT_Entry _has_targets_entry;
    vector<NT_Entry> _tx_entries;
    vector<NT_Entry> _ty_entries;
    vector<NT_Entry> _thor_entries;
    vector<NT_Entry> _tvert_entries;

    // Publishers
    image_transport::CameraPublisher _camera_pub;
    ros::Publisher _limelight_target_pub;

    // Subscribers
    ros::Subscriber _limelight_led_mode_sub;
    ros::Subscriber _limelight_cam_mode_sub;
    ros::Subscriber _limelight_info_sub;

    // Callbacks
    void led_mode_callback(const std_msgs::BoolConstPtr& msg);
    void cam_mode_callback(const std_msgs::BoolConstPtr& msg);

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
    double getDouble(NT_Entry entry, double default_value);
    void set_led_mode(bool mode);
    void publish_limelight_targets();
};
