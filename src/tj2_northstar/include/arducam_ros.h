#include <vector>
#include <boost/array.hpp>
#include <boost/range/algorithm.hpp>
#include <yaml-cpp/yaml.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/CameraInfo.h>
#include <tj2_interfaces/CameraInfoArray.h>

#include "arducam.h"

template<size_t Size, class Container>
boost::array<typename Container::value_type, Size> as_array(const Container &cont);

bool does_file_exist(const std::string& path);

class SplitCam
{
private:
    ros::NodeHandle *nh;
    std::string _prefix;
    int _index;
    image_transport::ImageTransport *_transport;
    sensor_msgs::CameraInfo _info;
    bool _info_did_load;

    sensor_msgs::ImagePtr _image_msg;
    cv_bridge::CvImagePtr _bridge_msg;

    image_transport::CameraPublisher _pub;

    bool load_camera_info(std::string path);
    std::string get_serial();
    std::string get_optical_frame();

public:
    SplitCam(
        std::string info_directory,
        std::string prefix,
        int index,
        ros::NodeHandle* nodehandle,
        image_transport::ImageTransport* transport);
    void process_image(cv::Mat image, ros::Time timestamp);
    sensor_msgs::CameraInfo get_info() { return _info; }
    ~SplitCam();
};


class ArducamROS
{
private:
    ros::NodeHandle nh;
    static const int NUM_CAMERAS = 4;
    SplitCam* cameras[NUM_CAMERAS];
    Arducam* _arducam;
    std::string _prefix;
    tj2_interfaces::CameraInfoArray _info_array;
    double _publish_rate;

    int _frame_rate;
    int _frame_timeout;
    bool _low_latency_mode;
    int _exposure;
    int _analogue_gain;

    image_transport::ImageTransport _image_transport;
    image_transport::Publisher _combined_pub;
    ros::Publisher _info_array_pub;

    void set_camera_parameters();
    
public:
    ArducamROS(ros::NodeHandle* node_handle);
    int run();
    ~ArducamROS();
};
