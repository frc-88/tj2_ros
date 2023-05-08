#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <apriltag_ros/common_functions.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <tj2_interfaces/CameraInfoArray.h>


class ApriltagCamArray
{
private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    std::shared_ptr<apriltag_ros::TagDetector> _tag_detector;
    
    image_transport::ImageTransport _image_transport;
    image_transport::Subscriber _array_sub;
    ros::Subscriber _info_sub;
    ros::Publisher _tag_detections_publisher;
    
    std::vector<sensor_msgs::CameraInfo> _info_array;
    std::map<int, apriltag_ros::StandaloneTagDescription> _standalone_tag_descriptions;
    std::vector<apriltag_ros::TagBundleDescription > _tag_bundle_descriptions;

    void image_callback(const sensor_msgs::ImageConstPtr& msg);
    void info_callback(const tj2_interfaces::CameraInfoArray& msg);
    apriltag_ros::AprilTagDetectionArray process_image(cv::Mat image, sensor_msgs::CameraInfo info);
    
public:
    ApriltagCamArray(ros::NodeHandle* node_handle, ros::NodeHandle* private_node_handle);
    int run();
    ~ApriltagCamArray();
};
