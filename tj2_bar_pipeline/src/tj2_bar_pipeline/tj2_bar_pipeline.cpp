#include "tj2_bar_pipeline/tj2_bar_pipeline.h"


TJ2BarPipeline::TJ2BarPipeline(ros::NodeHandle* nodehandle) :
    nh(*nodehandle),
    _image_transport(nh)
{
    ros::param::param<string>("~depth_topic", _depth_topic, "depth/image_raw");
    ros::param::param<string>("~color_topic", _color_topic, "color/image_raw");
    ros::param::param<string>("~color_info_topic", _color_info_topic, "color/camera_info");

    // Publishers
    _pipeline_debug_pub = _image_transport.advertise("pipeline_debug", 2);

    // Subscribers
    color_info_sub = nh.subscribe<CameraInfo>(_color_info_topic, 10, &TJ2BarPipeline::info_callback, this);
    color_sub.subscribe(nh, _color_topic, 2);
    depth_sub.subscribe(nh, _depth_topic, 2);

    sync.reset(new Sync(ExactSyncPolicy(1), color_sub, depth_sub));
    sync->registerCallback(boost::bind(&TJ2BarPipeline::rgbd_callback, this, _1, _2));

    ROS_INFO("tj2_bar_pipeline init done");
}

void TJ2BarPipeline::info_callback(const CameraInfoConstPtr& color_info)
{
    _camera_model.fromCameraInfo(color_info);
}

void TJ2BarPipeline::rgbd_callback(const ImageConstPtr& color_image, const ImageConstPtr& depth_image)
{
    ROS_ASSERT_MSG(color_image->width != 0, "Color image width is zero!");
    ROS_ASSERT_MSG(color_image->height != 0, "Color image height is zero!");

    ROS_ASSERT_MSG(depth_image->width != 0, "Depth image width is zero!");
    ROS_ASSERT_MSG(depth_image->height != 0, "Depth image height is zero!");

    // ---
    // Bridge color image
    // ---
    cv_bridge::CvImagePtr color_cv_ptr;
    try {
        color_cv_ptr = cv_bridge::toCvCopy(color_image, image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat color_cv_image;
    color_cv_image = color_cv_ptr->image;

    // ---
    // Bridge depth image
    // ---
    cv_bridge::CvImagePtr depth_cv_ptr;
    try {
        depth_cv_ptr = cv_bridge::toCvCopy(depth_image);  // encoding: passthrough
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat depth_cv_image, depth_cv_image_u8;
    depth_cv_image = depth_cv_ptr->image;
    depth_cv_image.convertTo(depth_cv_image_u8, CV_8U);

    GaussianBlur(depth_cv_image_u8, depth_cv_image_u8, Size(9, 9), 0);

    Mat dst, cdst, cdstP;
    // Edge detection
    Canny(depth_cv_image_u8, dst, 50, 200, 3);

    // Copy edges to the images that will display the results in BGR
    cvtColor(dst, cdstP, COLOR_GRAY2BGR);

    // cdst = cdstP.clone();
    // // Standard Hough Line Transform
    // vector<Vec2f> lines; // will hold the results of the detection
    // HoughLines(dst, lines, 1, CV_PI/180, 150, 0, 0 ); // runs the actual detection
    // // Draw the lines
    // for (size_t i = 0; i < lines.size(); i++)
    // {
    //     float rho = lines[i][0], theta = lines[i][1];
    //     Point pt1, pt2;
    //     double a = cos(theta), b = sin(theta);
    //     double x0 = a*rho, y0 = b*rho;
    //     pt1.x = cvRound(x0 + 1000*(-b));
    //     pt1.y = cvRound(y0 + 1000*(a));
    //     pt2.x = cvRound(x0 - 1000*(-b));
    //     pt2.y = cvRound(y0 - 1000*(a));
    //     line(cdst, pt1, pt2, Scalar(0, 0, 255), 3, LINE_AA);
    // }

    // Probabilistic Line Transform
    vector<Vec4i> linesP; // will hold the results of the detection
    HoughLinesP(dst, linesP, 1, CV_PI/180, 50, 50, 10 ); // runs the actual detection
    // Draw the lines
    for (size_t i = 0; i < linesP.size(); i++)
    {
        Vec4i l = linesP[i];
        line(cdstP, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0, 0, 255), 3, LINE_AA);
    }

    cv::Mat debug_img;
    // cv::vconcat(cdst, cdstP, debug_img);
    cv::vconcat(color_cv_image, cdstP, debug_img);

    ROS_INFO_STREAM("linesP.size(): " << linesP.size());

    if (_pipeline_debug_pub.getNumSubscribers() > 0)
    {
        // populate the message
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), image_encodings::BGR8, debug_img).toImageMsg();

        // publish the message
        _pipeline_debug_pub.publish(msg);
    }
}

int TJ2BarPipeline::run()
{
    ros::spin();
    return 0;
}
