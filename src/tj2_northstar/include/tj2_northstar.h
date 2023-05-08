#include <vector>
#include <boost/array.hpp>
#include <boost/range/algorithm.hpp>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <apriltag_ros/common_functions.h>

#include <sensor_msgs/CameraInfo.h>

#include "arducam.h"

template<size_t Size, class Container>
boost::array<typename Container::value_type, Size> as_array(const Container &cont);

bool does_file_exist(const std::string& path);
sensor_msgs::CompressedImagePtr image_to_compressed_message(cv::Mat image);

class SplitCam
{
private:
    ros::NodeHandle *nh;
    std::string _prefix;
    int _index;
    image_transport::ImageTransport *_transport;
    sensor_msgs::CameraInfo _info;
    bool _info_did_load;
    apriltag_ros::TagDetector* _tag_detector;

    sensor_msgs::ImagePtr _image_msg;
    cv_bridge::CvImagePtr _bridge_msg;

    image_transport::CameraPublisher _pub;

    bool load_camera_info(std::string path);
    std::string get_serial();

public:
    SplitCam(
        std::string info_directory,
        std::string prefix,
        int index,
        ros::NodeHandle* nodehandle,
        image_transport::ImageTransport* transport,
        apriltag_ros::TagDetector* tag_detector);
    apriltag_ros::AprilTagDetectionArray process_image(cv::Mat image, ros::Time timestamp);
    ~SplitCam();
};


class TJ2Northstar
{
private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    static const int NUM_CAMERAS = 4;
    SplitCam* cameras[NUM_CAMERAS];
    Arducam* _arducam;
    std::shared_ptr<apriltag_ros::TagDetector> _tag_detector;
    std::string _prefix;

    image_transport::ImageTransport _image_transport;
    image_transport::CameraPublisher _combined_pub;
    ros::Publisher _tag_detections_publisher;
    
public:
    TJ2Northstar(ros::NodeHandle* node_handle, ros::NodeHandle* private_node_handle);
    int run();
    ~TJ2Northstar();
};
