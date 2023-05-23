#include "apriltag_cam_array.h"


ApriltagCamArray::ApriltagCamArray(ros::NodeHandle* node_handle, ros::NodeHandle* private_node_handle) :
    nh(*node_handle),
    pnh(*private_node_handle),
    _image_transport(nh)
{
    _array_sub = _image_transport.subscribe("image_raw", 1, &ApriltagCamArray::image_callback, this);
    _info_sub = nh.subscribe("info", 1, &ApriltagCamArray::info_callback, this);
    _tag_detector = std::shared_ptr<apriltag_ros::TagDetector>(new apriltag_ros::TagDetector(pnh));
    _tag_detections_publisher = nh.advertise<apriltag_ros::AprilTagDetectionArray>("tag_detections", 1);
}

ApriltagCamArray::~ApriltagCamArray()
{

}

void ApriltagCamArray::image_callback(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat image;
    try {
        image = cv_bridge::toCvShare(msg, "mono8")->image;
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
        return;
    }

    int num_cameras = _info_array.size();
    if (num_cameras == 0) {
        ROS_WARN("No cameras registered. Is anything being published on the info array topic?");
        return;
    }
    else {
        ROS_INFO_ONCE("Camera info received.");
    }
    cv::Size size = image.size();
    int sub_width = size.width / num_cameras;
    apriltag_ros::AprilTagDetectionArray all_tags;
    for (int index = 0; index < num_cameras; index++) {
        cv::Rect split(sub_width * index, 0, sub_width, size.height);
        apriltag_ros::AprilTagDetectionArray tags = process_image(image(split), _info_array.at(index));
        all_tags.header.stamp = tags.header.stamp;
        all_tags.detections.insert(all_tags.detections.end(), tags.detections.begin(), tags.detections.end());
    }
    _tag_detections_publisher.publish(all_tags);
}

void ApriltagCamArray::info_callback(const tj2_interfaces::CameraInfoArray& msg)
{
    _info_array = msg.cameras;
}

apriltag_ros::AprilTagDetectionArray ApriltagCamArray::process_image(cv::Mat image, sensor_msgs::CameraInfo info)
{
    sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(info.header, "mono8", image).toImageMsg();
    cv_bridge::CvImagePtr bridge_msg = cv_bridge::toCvCopy(image_msg, image_msg->encoding);
    return _tag_detector->detectTags(bridge_msg, sensor_msgs::CameraInfoConstPtr(new sensor_msgs::CameraInfo(info)));
}

int ApriltagCamArray::run() {
    ros::spin();
    return 0;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "apriltag_cam_array");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ApriltagCamArray node(&nh, &pnh);
    return node.run();
}
