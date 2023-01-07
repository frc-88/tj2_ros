#pragma once

#include "ntcore.h"
#include <boost/array.hpp>
#include <boost/range/algorithm.hpp>

#include <camera_info_manager/camera_info_manager.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/camera_publisher.h>
#include <image_transport/image_transport.h>

#include <image_geometry/pinhole_camera_model.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include "nav_msgs/Odometry.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>

#include <geometry_msgs/PoseStamped.h>

#include <std_msgs/Float64.h>

#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <boost/thread/thread.hpp>

#include <tj2_limelight/LimelightTargetArray.h>

#include <std_msgs/Bool.h>


using namespace std;


double to_radians(double degrees) {
    return degrees * M_PI / 180.0;
}


double inches_to_meters(double inches) {
    return inches * 0.0254;
}


template<size_t Size, class Container>
boost::array<typename Container::value_type, Size> as_array(const Container &cont)
{
    assert(cont.size() == Size);
    boost::array<typename Container::value_type, Size> result;
    boost::range::copy(cont, result.begin());
    return result;
}

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
    image_geometry::PinholeCameraModel _camera_model;
    image_transport::ImageTransport _image_transport;

    boost::thread* _watcher_thread;
    ros::Time _last_publish_time;

    NT_Inst _nt;

    // Parameters
    bool _publish_video;
    string _nt_host;
    int _nt_port;
    int _num_limelight_targets;
    string _video_url;
    string _camera_info_url;
    string _frame_id;
    double _max_frame_rate;
    string _target_frame;
    string _odom_frame;
    string _odom_child_frame;
    double _field_vision_target_height, _field_vision_target_distance;

    nav_msgs::Odometry _odom_msg;
    std::vector<double> _odom_covariance;
    std::vector<double> _twist_covariance;

    NT_Entry _limelight_led_mode_entry;
    NT_Entry _limelight_cam_mode_entry;
    NT_Entry _has_targets_entry;
    vector<NT_Entry> _tx_entries;
    vector<NT_Entry> _ty_entries;
    vector<NT_Entry> _thor_entries;
    vector<NT_Entry> _tvert_entries;
    NT_Entry _main_tx_entry;
    NT_Entry _main_ty_entry;

    NT_Entry _limelight_target_angle_entry;
    NT_Entry _limelight_target_distance_entry;

    // Publishers
    image_transport::CameraPublisher _camera_pub;
    ros::Publisher _limelight_raw_target_pub;
    ros::Publisher _limelight_target_pub;
    ros::Publisher _limelight_target_angle_pub;
    ros::Publisher _limelight_target_distance_pub;
    ros::Publisher _odom_pub;

    // Subscribers
    ros::Subscriber _limelight_led_mode_sub;
    ros::Subscriber _limelight_cam_mode_sub;
    ros::Subscriber _limelight_info_sub;

    // Callbacks
    void led_mode_callback(const std_msgs::BoolConstPtr& msg);
    void cam_mode_callback(const std_msgs::BoolConstPtr& msg);

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
    vector<double> get_double_list_param(string name, size_t length);
};
