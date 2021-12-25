#pragma once

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/PoseStamped.h>

#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Header.h>

#include <vision_msgs/Detection2D.h>
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/BoundingBox2D.h>

#include <tj2_limelight/LimelightTarget.h>

#include <image_geometry/pinhole_camera_model.h>

#include <jetson-inference/imageNet.h>


using namespace std;
using namespace sensor_msgs;

#define THROW_EXCEPTION(msg)  throw std::runtime_error(msg)


class LimelightTargetNode
{
public:
    LimelightTargetNode(ros::NodeHandle* nodehandle);
    int run();
private:
    ros::NodeHandle nh;  // ROS node handle

    // Members
    image_transport::ImageTransport _image_transport;
    image_geometry::PinholeCameraModel _camera_model;
    sensor_msgs::CameraInfo _camera_info;
    std::map<std::string, std_msgs::ColorRGBA> _marker_colors;
    double _min_contour_area, _max_contour_area;
    int _approx_sync_queue_size;

    cv::Scalar hsv_lower_bound;
    cv::Scalar hsv_upper_bound;

    std::vector<std::string> _class_descriptions;
    std::map<std::string, double> _z_depth_estimations;
    uint32_t _num_classes;

    // Publishers
    ros::Publisher _detection_array_pub;
    ros::Publisher _marker_pub;
    ros::Publisher _target_detection_pub;
    image_transport::Publisher _pipeline_pub;

    // Subscribers
    message_filters::Subscriber<Image>_color_sub;
    message_filters::Subscriber<Image>_depth_sub;
    message_filters::Subscriber<CameraInfo> _depth_info_sub;
    message_filters::Subscriber<tj2_limelight::LimelightTarget> _target_sub;

    // imageNet variables
    imageNet* _net;

    // ROS TF
    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener;

    // Topic synchronization
    typedef message_filters::sync_policies::ApproximateTime<Image, CameraInfo, tj2_limelight::LimelightTarget> TargetApproxSyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<Image, Image, CameraInfo> CameraApproxSyncPolicy;
    typedef message_filters::Synchronizer<TargetApproxSyncPolicy> TargetSync;
    typedef message_filters::Synchronizer<CameraApproxSyncPolicy> CameraSync;
    boost::shared_ptr<TargetSync> target_sync;
    boost::shared_ptr<CameraSync> camera_sync;

    // Parameters
    double _text_marker_size;
    string _target_frame;
    double _marker_persistance_s;
    ros::Duration _marker_persistance;
    vector<int> _hsv_lower_bound_param;
    vector<int> _hsv_upper_bound_param;
    XmlRpc::XmlRpcValue _marker_colors_param;

    string _model_name;
    string _model_path;
    string _prototxt_path;
    string _class_labels_path;
    string _input_blob;
    string _output_blob;
    double _threshold;

    bool msg_to_frame(const ImageConstPtr msg, cv::Mat& image);
    vision_msgs::Detection2D target_to_detection(int class_index, cv::Mat depth_cv_image, cv::Rect bndbox, cv::Point3d& dimensions);
    double get_target_z(cv::Mat depth_cv_image, cv::Rect target);
    void publish_markers(string name, int index, vision_msgs::Detection2D det_msg, cv::Point3d dimensions);
    visualization_msgs::Marker make_marker(string name, int index, vision_msgs::Detection2D det_msg, cv::Point3d dimensions);
    int classify(cv::Mat cropped_image);
    void detection_pipeline(cv::Mat frame, vector<cv::Rect>* detection_boxes, vector<int>* classes);

    void load_imagenet_model();
    void load_labels();

    // Callbacks
    void target_callback(const ImageConstPtr& depth_image, const CameraInfoConstPtr& depth_info, const tj2_limelight::LimelightTargetConstPtr& target);
    void camera_callback(const ImageConstPtr& color_image, const ImageConstPtr& depth_image, const CameraInfoConstPtr& depth_info);
};