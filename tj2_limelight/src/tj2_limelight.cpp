#include "tj2_limelight.h"

TJ2Limelight::TJ2Limelight(ros::NodeHandle* nodehandle) :
    nh(*nodehandle),
    _image_transport(nh),
    _camera_info_manager(nh)
{
  _camera_pub = _image_transport.advertiseCamera("camera/image", 10);
    ros::param::param<string>("~video_url", _video_url, "");
    ros::param::param<string>("~camera_info_url", _camera_info_url, "");
    ros::param::param<string>("~frame_id", _frame_id, "camera_link");
    ros::param::param<double>("~max_frame_rate", _max_frame_rate, 30.0);

    _camera_info_manager.setCameraName("limelight");

    if (_camera_info_manager.validateURL(_camera_info_url))
    {
        if (_camera_info_manager.loadCameraInfo(_camera_info_url)) {
            ROS_INFO_STREAM("Loaded camera calibration from " << _camera_info_url);
        }
        else {
            ROS_WARN_STREAM("Could not load camera info, using an uncalibrated config.");
        }
    }
    else {
        ROS_WARN_STREAM("Given camera info url: " << _camera_info_url << " is not valid, using an uncalibrated config.");
    }

    ROS_INFO_STREAM("Trying to connect to  " << _video_url);
    _video_capture.open(_video_url);

    _out_msg.header.frame_id = _frame_id;
    _out_msg.encoding = sensor_msgs::image_encodings::BGR8;

    _camera_info = _camera_info_manager.getCameraInfo();
    _camera_info.header.frame_id = _frame_id;

    _reopenSleep = ros::Duration(0.25);
    
    _watcher_thread = new boost::thread(&TJ2Limelight::watchVideoCapture, this);

}
void TJ2Limelight::watchVideoCapture()
{
    ros::Rate clock_rate(_max_frame_rate);  // Hz
    ros::Duration timeout(10.0);
    while (ros::ok())
    {
        clock_rate.sleep();
        if (_camera_pub.getNumSubscribers() <= 0) {
            continue;
        }
        if (ros::Time::now() - _last_publish_time > timeout) {
            reopenCapture();
        }
    }
}

int TJ2Limelight::run()
{
    cv::Mat frame;
    ros::Rate loop(_max_frame_rate);
    while (ros::ok())
    {
        if (_video_capture.isOpened())
        {
            ROS_INFO_ONCE("connection established");
            loop.sleep();
            ros::spinOnce();
            // if (_camera_pub.getNumSubscribers() > 0)
            // {
                // ROS_INFO_ONCE("publishing frames");
            if (!_video_capture.read(frame)) {
                continue;
            }
            _last_publish_time = ros::Time::now();
            _out_msg.header.stamp = _last_publish_time;
            _out_msg.image = frame;

            _camera_info.header.stamp = _last_publish_time;

            _out_msg.toImageMsg(_ros_img);
            _camera_pub.publish(_ros_img, _camera_info, _last_publish_time);
            // }
        }
        else
        {
            reopenCapture();
        }

        ros::spinOnce();
        loop.sleep();
    }
    return 0;
}

void TJ2Limelight::reopenCapture()
{
    ROS_WARN("Video stream is not available, retrying...");
    _video_capture.release();
    exit(0);
    // ROS_WARN_STREAM_DELAYED_THROTTLE(2, "Video stream is not available, retrying...");
    // _reopenSleep.sleep();
    // _video_capture.open(_video_url);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tj2_limelight");
    ros::NodeHandle nh;
    TJ2Limelight node(&nh);
    return node.run();
}
