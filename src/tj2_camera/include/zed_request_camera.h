#include <sl/Camera.hpp>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/distortion_models.h>
#include <tj2_interfaces/RequestFrames.h>

class ZedRequestCamera
{
private:
    ros::NodeHandle nh; // ROS node handle
    sl::Camera _zed;
    sl::InitParameters _init_parameters;
    sl::RuntimeParameters _run_parameters;
    sl::Resolution _resolution;

    std::string _image_frame_id;
    ros::Time _request_time;
    ros::Duration _request_duration;
    bool _depth_enabled;
    double _tick_rate;

    image_transport::CameraPublisher _image_pub;
    image_transport::CameraPublisher _depth_pub;
    image_transport::ImageTransport _image_transport;
    ros::Subscriber _request_sub;

    sensor_msgs::CameraInfoPtr _camera_info;

    void request_callback(const tj2_interfaces::RequestFrames::ConstPtr &msg);
    void init_camera_info();
    bool poll();
    int init_camera();

public:
    ZedRequestCamera(ros::NodeHandle *nodehandle);
    int run();
};

ros::Time sl_time_to_ros(sl::Timestamp timestamp)
{
    uint32_t sec = static_cast<uint32_t>(timestamp.getNanoseconds() / 1000000000);
    uint32_t nsec = static_cast<uint32_t>(timestamp.getNanoseconds() % 1000000000);
    return ros::Time(sec, nsec);
}

void image_to_ros_msg(sensor_msgs::ImagePtr imgMsgPtr, sl::Mat img, std::string frameId, ros::Time t)
{
    if (!imgMsgPtr)
    {
        return;
    }

    imgMsgPtr->header.stamp = t;
    imgMsgPtr->header.frame_id = frameId;
    imgMsgPtr->height = img.getHeight();
    imgMsgPtr->width = img.getWidth();

    int num = 1; // for endianness detection
    imgMsgPtr->is_bigendian = !(*(char *)&num == 1);

    imgMsgPtr->step = img.getStepBytes();

    size_t size = imgMsgPtr->step * imgMsgPtr->height;
    imgMsgPtr->data.resize(size);

    sl::MAT_TYPE dataType = img.getDataType();

    switch (dataType)
    {
    case sl::MAT_TYPE::F32_C1: /**< float 1 channel.*/
        imgMsgPtr->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        memcpy((char *)(&imgMsgPtr->data[0]), img.getPtr<sl::float1>(), size);
        break;

    case sl::MAT_TYPE::F32_C2: /**< float 2 channels.*/
        imgMsgPtr->encoding = sensor_msgs::image_encodings::TYPE_32FC2;
        memcpy((char *)(&imgMsgPtr->data[0]), img.getPtr<sl::float2>(), size);
        break;

    case sl::MAT_TYPE::F32_C3: /**< float 3 channels.*/
        imgMsgPtr->encoding = sensor_msgs::image_encodings::TYPE_32FC3;
        memcpy((char *)(&imgMsgPtr->data[0]), img.getPtr<sl::float3>(), size);
        break;

    case sl::MAT_TYPE::F32_C4: /**< float 4 channels.*/
        imgMsgPtr->encoding = sensor_msgs::image_encodings::TYPE_32FC4;
        memcpy((char *)(&imgMsgPtr->data[0]), img.getPtr<sl::float4>(), size);
        break;

    case sl::MAT_TYPE::U8_C1: /**< unsigned char 1 channel.*/
        imgMsgPtr->encoding = sensor_msgs::image_encodings::MONO8;
        memcpy((char *)(&imgMsgPtr->data[0]), img.getPtr<sl::uchar1>(), size);
        break;

    case sl::MAT_TYPE::U8_C2: /**< unsigned char 2 channels.*/
        imgMsgPtr->encoding = sensor_msgs::image_encodings::TYPE_8UC2;
        memcpy((char *)(&imgMsgPtr->data[0]), img.getPtr<sl::uchar2>(), size);
        break;

    case sl::MAT_TYPE::U8_C3: /**< unsigned char 3 channels.*/
        imgMsgPtr->encoding = sensor_msgs::image_encodings::BGR8;
        memcpy((char *)(&imgMsgPtr->data[0]), img.getPtr<sl::uchar3>(), size);
        break;

    case sl::MAT_TYPE::U8_C4: /**< unsigned char 4 channels.*/
        imgMsgPtr->encoding = sensor_msgs::image_encodings::BGRA8;
        memcpy((char *)(&imgMsgPtr->data[0]), img.getPtr<sl::uchar4>(), size);
        break;

    case sl::MAT_TYPE::U16_C1: /**< unsigned short 1 channel.*/
        imgMsgPtr->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
        memcpy((uint16_t *)(&imgMsgPtr->data[0]), img.getPtr<sl::ushort1>(), size);
        break;
    }
}