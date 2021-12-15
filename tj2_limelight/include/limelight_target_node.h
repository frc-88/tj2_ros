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

#include <tj2_limelight/LimelightTarget.h>

#include <image_geometry/pinhole_camera_model.h>


using namespace std;
using namespace sensor_msgs;

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

    // Publishers
    ros::Publisher _marker_pub;
    ros::Publisher _target_pose_pub;

    // Subscribers
    message_filters::Subscriber<Image>_depth_sub;
    message_filters::Subscriber<CameraInfo> _depth_info_sub;
    message_filters::Subscriber<tj2_limelight::LimelightTarget> _target_sub;

    // ROS TF
    tf2_ros::Buffer _tf_buffer;
    tf2_ros::TransformListener _tf_listener;

    // Topic synchronization
    typedef message_filters::sync_policies::ApproximateTime<Image, CameraInfo, tj2_limelight::LimelightTarget> ApproxSyncPolicy;
    typedef message_filters::Synchronizer<ApproxSyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

    // Parameters
    double _text_marker_size;
    string _target_frame;
    double _marker_persistance_s;
    ros::Duration _marker_persistance;

    double get_target_z(cv::Mat depth_cv_image, tj2_limelight::LimelightTargetConstPtr target);
    void publish_target_pose(geometry_msgs::PoseStamped object_pose, const CameraInfoConstPtr depth_info, cv::Point3d center);
    visualization_msgs::Marker make_marker(geometry_msgs::PoseStamped object_pose, cv::Point3d center, cv::Point3d edge);
    void publish_markers(geometry_msgs::PoseStamped object_pose, cv::Point3d center, cv::Point3d edge);

    // Callbacks
    void target_callback(const ImageConstPtr& depth_image, const CameraInfoConstPtr& depth_info, const tj2_limelight::LimelightTargetConstPtr& target);
};
