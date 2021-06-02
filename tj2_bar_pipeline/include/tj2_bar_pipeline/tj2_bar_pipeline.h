#pragma once

#include <exception>
#include <iostream>
#include <ctime>
#include <math.h>

#include "ros/ros.h"
#include "ros/console.h"

#include <xmlrpcpp/XmlRpcValue.h>

#include <std_msgs/ColorRGBA.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <image_geometry/pinhole_camera_model.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>


using namespace cv;
using namespace std;
using namespace sensor_msgs;

class TJ2BarPipeline
{
private:
    ros::NodeHandle nh;  // ROS node handle

    // launch parameters
    string _color_topic;
    string _color_info_topic;
    string _depth_topic;

    // Object properties
    image_geometry::PinholeCameraModel _camera_model;

    // Subscribers
    ros::Subscriber color_info_sub;
    message_filters::Subscriber<Image> color_sub;
    message_filters::Subscriber<Image> depth_sub;
    
    typedef message_filters::sync_policies::ExactTime<Image, Image> ExactSyncPolicy;
    typedef message_filters::Synchronizer<ExactSyncPolicy> Sync;
    boost::shared_ptr<Sync> sync;

    // Publishers
    image_transport::Publisher _pipeline_debug_pub;
    image_transport::ImageTransport _image_transport;

    // Sub callbacks
    void rgbd_callback(const ImageConstPtr& color_image, const ImageConstPtr& depth_image);
    void info_callback(const CameraInfoConstPtr& color_info);

public:
    TJ2BarPipeline(ros::NodeHandle* nodehandle);
    int run();
};
