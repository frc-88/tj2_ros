#include "tj2_depth_converter.h"

TJ2DepthConverter::TJ2DepthConverter(ros::NodeHandle* nodehandle) :
    nh(*nodehandle),
    _image_transport(nh)
{
    ros::param::param<int>("~erosion_size", _erosion_size, 3);
    ros::param::param<double>("~throttle_frame_rate", _throttle_frame_rate, 10.0);
    ros::param::param<double>("~connected_components_size_threshold", _connected_components_size_threshold, 500.0);

    _erode_element = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(2 * _erosion_size + 1, 2 * _erosion_size + 1),
        cv::Point(_erosion_size, _erosion_size)
    );
    _depth_info_set = false;
    _info_sub = nh.subscribe<sensor_msgs::CameraInfo>("depth/camera_info", 10, &TJ2DepthConverter::infoCallback, this);

    _depth_sub = _image_transport.subscribe("depth/image_raw", 10, &TJ2DepthConverter::depthCallback, this);
    _depth_filtered_pub = _image_transport.advertiseCamera("depth_filtered", 10);
    _prev_frame_time = ros::Time::now();
    _prev_frame_delay = ros::Duration(1.0 / _throttle_frame_rate);
}

void TJ2DepthConverter::infoCallback(const sensor_msgs::CameraInfoConstPtr& depth_info)
{
    _depth_info = *depth_info;
    _depth_info_set = true;
}

void TJ2DepthConverter::depthCallback(const sensor_msgs::ImageConstPtr& depth_image)
{
    if (!_depth_info_set) {
        return;
    }
    if (ros::Time::now() - _prev_frame_time < _prev_frame_delay) {
        return;
    }
    _prev_frame_time = ros::Time::now();
    cv_bridge::CvImagePtr depth_cv_ptr;
    try {
        depth_cv_ptr = cv_bridge::toCvCopy(depth_image);  // encoding: passthrough
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge parameter exception: %s", e.what());
        return;
    }

    cv::Mat depth_cv_image, depth_cv_image_u8;
    depth_cv_image = depth_cv_ptr->image;
    depth_cv_image.convertTo(depth_cv_image_u8, CV_8U, 1.0/256.0);
    cv::threshold(depth_cv_image_u8, depth_cv_image_u8, 1, 255, cv::THRESH_BINARY);
    cv::Mat labels, stats, centroid;
    int ret = cv::connectedComponentsWithStats(depth_cv_image_u8, labels, stats, centroid);
    cv::Mat mask(depth_cv_image_u8.rows, depth_cv_image_u8.cols, CV_8UC1, cv::Scalar(255));
    for (int i = 0; i < stats.rows; i++)
    {
        double size = stats.at<double>(cv::Point(4, i));
        if (size > _connected_components_size_threshold) {
            continue;
        }

        int x = stats.at<int>(cv::Point(0, i));
        int y = stats.at<int>(cv::Point(1, i));
        int w = stats.at<int>(cv::Point(2, i));
        int h = stats.at<int>(cv::Point(3, i));

        cv::Rect rect(x, y, w, h);
        cv::rectangle(mask, rect, cv::Scalar(0));
    }
    depth_cv_image.copyTo(depth_cv_image, mask);

    cv::erode(depth_cv_image, depth_cv_image, _erode_element);
    cv::dilate(depth_cv_image, depth_cv_image, _erode_element);

    sensor_msgs::ImagePtr msg;
    try {
        msg = cv_bridge::CvImage(depth_image->header, sensor_msgs::image_encodings::MONO16, depth_cv_image).toImageMsg();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge publish exception: %s", e.what());
        return;
    }
    _depth_filtered_pub.publish(*msg, _depth_info);
}

int TJ2DepthConverter::run()
{
    ros::spin();
    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tj2_depth_converter");
    ros::NodeHandle nh;
    TJ2DepthConverter node(&nh);
    return node.run();
}
