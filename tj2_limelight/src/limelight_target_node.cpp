#include "limelight_target_node.h"

LimelightTargetNode::LimelightTargetNode(ros::NodeHandle* nodehandle) :
    nh(*nodehandle),
    _image_transport(nh),
    _tf_listener(_tf_buffer)
{
    ros::param::param<double>("~text_marker_size", _text_marker_size, 0.25);
    ros::param::param<string>("~target_frame", _target_frame, "base_link");
    ros::param::param<double>("~marker_persistance_s", _marker_persistance_s, 0.1);
    ros::param::param<double>("~min_contour_area", _min_contour_area, 0.0);
    ros::param::param<double>("~max_contour_area", _max_contour_area, 1000.0);
    ros::param::param<int>("~approx_sync_queue_size", _approx_sync_queue_size, 10);
    _marker_persistance = ros::Duration(_marker_persistance_s);

    string key;
    if (!ros::param::search("hsv_lower_bound", key)) {
        THROW_EXCEPTION("Failed to find hsv_lower_bound parameter");
    }
    ROS_DEBUG("Found hsv_lower_bound: %s", key.c_str());
    nh.getParam(key, _hsv_lower_bound_param);

    if (_hsv_lower_bound_param.size() != 3) {
        ROS_ERROR("hsv_upper_bound is size %lu", _hsv_lower_bound_param.size());
        THROW_EXCEPTION("hsv_lower_bound parameter is the incorrect size");
    }

    if (!ros::param::search("hsv_upper_bound", key)) {
        THROW_EXCEPTION("Failed to find hsv_upper_bound parameter");
    }
    ROS_DEBUG("Found hsv_upper_bound: %s", key.c_str());
    nh.getParam(key, _hsv_upper_bound_param);

    if (_hsv_upper_bound_param.size() != 3) {
        ROS_ERROR("hsv_upper_bound is size %lu", _hsv_upper_bound_param.size());
        THROW_EXCEPTION("hsv_upper_bound parameter is the incorrect size");
    }

    hsv_lower_bound = cv::Scalar(
        _hsv_lower_bound_param.at(0),
        _hsv_lower_bound_param.at(1),
        _hsv_lower_bound_param.at(2)
    );
    hsv_upper_bound = cv::Scalar(
        _hsv_upper_bound_param.at(0),
        _hsv_upper_bound_param.at(1),
        _hsv_upper_bound_param.at(2)
    );

    if (ros::param::search("marker_colors", key))
    {
        ROS_DEBUG("Found marker_colors: %s", key.c_str());
        nh.getParam(key, _marker_colors_param);

        // _marker_colors_param is an array
        if (_marker_colors_param.getType() != XmlRpc::XmlRpcValue::Type::TypeArray ||
            _marker_colors_param.size() == 0) {
            THROW_EXCEPTION("marker_colors wrong type or size");
        }
        ROS_DEBUG("marker_colors is the correct type");

        _marker_colors = new std::vector<std_msgs::ColorRGBA>(_marker_colors_param.size());
        for (size_t index = 0; index < _marker_colors_param.size(); index++)
        {
            if (_marker_colors_param[index].getType() != XmlRpc::XmlRpcValue::TypeArray) {
                THROW_EXCEPTION("marker_colors element is not a list");
            }
            ROS_DEBUG("\tFound marker_colors entry");

            std_msgs::ColorRGBA color;
            color.r = (double)(_marker_colors_param[index][0]);
            color.g = (double)(_marker_colors_param[index][1]);
            color.b = (double)(_marker_colors_param[index][2]);
            color.a = (double)(_marker_colors_param[index][3]);
            _marker_colors->at(index) = color;
            ROS_DEBUG("\tR=%0.2f, G=%0.2f, B=%0.2f, A=%0.2f", color.r, color.g, color.b, color.a);
        }
    }
    else {
        _marker_colors = new std::vector<std_msgs::ColorRGBA>(0);
    }
    
    _pipeline_pub = _image_transport.advertise("pipeline/image_raw", 2);

    _marker_pub = nh.advertise<visualization_msgs::MarkerArray>("pipeline/markers", 10);
    _target_detection_pub = nh.advertise<vision_msgs::Detection2D>("detections/target", 10);
    _detection_array_pub = nh.advertise<vision_msgs::Detection2DArray>("detections/array", 10);

    _color_sub.subscribe(nh, "color/image_raw", 10);
    _depth_sub.subscribe(nh, "depth/image_raw", 10);
    _depth_info_sub.subscribe(nh, "depth/camera_info", 10);
    _target_sub.subscribe(nh, "/limelight/target", 10);

    target_sync.reset(new TargetSync(TargetApproxSyncPolicy(_approx_sync_queue_size), _depth_sub, _depth_info_sub, _target_sub));
    target_sync->registerCallback(boost::bind(&LimelightTargetNode::target_callback, this, _1, _2, _3));

    camera_sync.reset(new CameraSync(CameraApproxSyncPolicy(_approx_sync_queue_size), _color_sub, _depth_sub, _depth_info_sub));
    camera_sync->registerCallback(boost::bind(&LimelightTargetNode::camera_callback, this, _1, _2, _3));
}

void LimelightTargetNode::camera_callback(const ImageConstPtr& color_image, const ImageConstPtr& depth_image, const CameraInfoConstPtr& depth_info)
{
    _camera_model.fromCameraInfo(depth_info);
    
    cv::Mat color_cv_image;
    if (!msg_to_frame(color_image, color_cv_image)) {
        return;
    }
    cv::Mat depth_cv_image;
    if (!msg_to_frame(depth_image, depth_cv_image)) {
        return;
    }
    vector<cv::Rect>* detection_boxes = new vector<cv::Rect>();
    detection_pipeline(color_cv_image, detection_boxes);
    
    vision_msgs::Detection2DArray det_array_msg;
    det_array_msg.header = color_image->header;
    for (size_t index = 0; index < detection_boxes->size(); index++) {
        cv::Rect bndbox = detection_boxes->at(index);
        cv::Point3d dimensions;
        vision_msgs::Detection2D det_msg = target_to_detection(index, depth_cv_image, bndbox, dimensions);

        // if (index == 0) {
        //     det_msg.source_img = *color_image;
        // }
        det_msg.header.stamp = depth_image->header.stamp;
        det_msg.header.frame_id = _camera_model.tfFrame();

        det_array_msg.detections.push_back(det_msg);
        publish_markers("limelight", index + 1, det_msg, dimensions);
    }
    _detection_array_pub.publish(det_array_msg);
}

void LimelightTargetNode::target_callback(const ImageConstPtr& depth_image, const CameraInfoConstPtr& depth_info, const tj2_limelight::LimelightTargetConstPtr& target)
{
    if (!target->tv) {
        return;
    }

    _camera_model.fromCameraInfo(depth_info);

    cv::Mat depth_cv_image;
    if (!msg_to_frame(depth_image, depth_cv_image)) {
        return;
    }
    
    cv::Rect bndbox(
        target->tx - target->thor / 2,
        target->ty - target->tvert / 2,
        target->thor, target->tvert
    );
    cv::Point3d dimensions;
    vision_msgs::Detection2D det_msg = target_to_detection(0, depth_cv_image, bndbox, dimensions);

    det_msg.header.stamp = depth_image->header.stamp;
    det_msg.header.frame_id = _camera_model.tfFrame();

    _target_detection_pub.publish(det_msg);
    publish_markers("target", 0, det_msg, dimensions);
}

vision_msgs::Detection2D LimelightTargetNode::target_to_detection(int id, cv::Mat depth_cv_image, cv::Rect bndbox, cv::Point3d& dimensions)
{
    vision_msgs::Detection2D det_msg;
    int width = bndbox.width;
    int height = bndbox.height;
    int cx = bndbox.x + width / 2;
    int cy = bndbox.y + height / 2;
    
    // Project limelight target onto depth image
    cv::Point2d target_point;
    target_point.x = cx;
    target_point.y = cy;
    cv::Point3d ray = _camera_model.projectPixelTo3dRay(target_point);

    double z_dist = get_target_z(depth_cv_image, bndbox);
    
    cv::Point3d center;
    center.x = ray.x * z_dist;
    center.y = ray.y * z_dist;
    center.z = z_dist;

    ROS_DEBUG("center, X: %0.3f, Y: %0.3f, Z: %0.3f", center.x, center.y, center.z);

    cv::Point2d edge_point;
    edge_point.x = cx + width / 2;
    edge_point.y = cy + height / 2;
    ray = _camera_model.projectPixelTo3dRay(edge_point);
    
    cv::Point3d edge;
    edge.x = ray.x * z_dist;
    edge.y = ray.y * z_dist;
    edge.z = z_dist;
    ROS_DEBUG("edge,   X: %0.3f, Y: %0.3f, Z: %0.3f", edge.x, edge.y, edge.z);

    dimensions.x = 2.0 * (edge.x - center.x);
    dimensions.y = 2.0 * (edge.y - center.y);
    dimensions.z = 0.005;  // We have no information on how deep the target goes.

    geometry_msgs::Pose object_pose;
    object_pose.position.x = center.x;
    object_pose.position.y = center.y;
    object_pose.position.z = center.z;
    object_pose.orientation.x = 0.0;
    object_pose.orientation.y = 0.0;
    object_pose.orientation.z = 0.0;
    object_pose.orientation.w = 1.0;

    det_msg.bbox.center.x = bndbox.x;
    det_msg.bbox.center.y = bndbox.y;
    det_msg.bbox.size_x = bndbox.width;
    det_msg.bbox.size_y = bndbox.height;

    vision_msgs::ObjectHypothesisWithPose hypothesis;

    hypothesis.id = id;
    hypothesis.score = 0.0;
    hypothesis.pose.pose = object_pose;

    det_msg.results.push_back(hypothesis);

    return det_msg;
}


bool LimelightTargetNode::msg_to_frame(const ImageConstPtr msg, cv::Mat& image)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg);  // encoding: passthrough
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return false;
    }
    if (msg->width == 0 || msg->height == 0) {
        ROS_ERROR("Image has a zero width dimension!");
        return false;
    }
    image = cv_ptr->image;
    return true;
}

void LimelightTargetNode::publish_markers(string name, int index, vision_msgs::Detection2D det_msg, cv::Point3d dimensions)
{
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker rect_marker = make_marker(name, index, det_msg, dimensions);
    visualization_msgs::Marker text_marker = make_marker(name, index, det_msg, dimensions);

    rect_marker.type = visualization_msgs::Marker::CUBE;
    rect_marker.ns = "cube_" + rect_marker.ns;

    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.text = text_marker.ns;
    text_marker.ns = "text_" + text_marker.ns;
    text_marker.pose.position.y += dimensions.y;  // Z is perpendicular to camera plane. Y is up in world coordinates here
    text_marker.scale.x = 0.0;
    text_marker.scale.y = 0.0;
    text_marker.scale.z = _text_marker_size;
    text_marker.color.r = 1.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 1.0;
    text_marker.color.a = 1.0;

    markers.markers.push_back(rect_marker);
    markers.markers.push_back(text_marker);

    _marker_pub.publish(markers);
}

visualization_msgs::Marker LimelightTargetNode::make_marker(string name, int index, vision_msgs::Detection2D det_msg, cv::Point3d dimensions)
{
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = det_msg.results[0].pose.pose;
    marker.header = det_msg.header;
    marker.lifetime = _marker_persistance;
    marker.ns = name;
    marker.id = det_msg.results[0].id;

    geometry_msgs::Vector3 scale_vector;
    scale_vector.x = dimensions.x;
    scale_vector.y = dimensions.y;
    scale_vector.z = dimensions.z;
    marker.scale = scale_vector;
    if (_marker_colors->size() == 0) {
        marker.color = std_msgs::ColorRGBA();
        marker.color.r = 1.0;
        marker.color.a = 0.5;
    }
    else {
        marker.color = _marker_colors->at(index % _marker_colors->size());
    }

    return marker;
}

double LimelightTargetNode::get_target_z(cv::Mat depth_cv_image, cv::Rect target)
{
    ROS_DEBUG("Depth image size: w=%d, h=%d", depth_cv_image.rows, depth_cv_image.cols);
    cv::Mat rectangle_mask = cv::Mat::zeros(depth_cv_image.rows, depth_cv_image.cols, CV_8UC1);
    cv::rectangle(rectangle_mask, target, cv::Scalar(255, 255, 255), cv::FILLED);
    ROS_DEBUG("Created rectangle mask");

    cv::Mat nonzero_mask = (depth_cv_image > 0.0);
    nonzero_mask.convertTo(nonzero_mask, CV_8U);
    ROS_DEBUG("Created nonzero mask");

    cv::Mat target_mask;
    cv::bitwise_and(rectangle_mask, nonzero_mask, target_mask);
    ROS_DEBUG("Created target mask");

    double z_dist = cv::mean(depth_cv_image, target_mask)[0];  // depth values are in mm
    ROS_DEBUG("z_dist mm: %f", z_dist);

    z_dist /= 1000.0;

    return z_dist;
}

void LimelightTargetNode::detection_pipeline(cv::Mat frame, vector<cv::Rect>* detection_boxes)
{
    cv::Mat frame_hsv, contour_image;
    vector<vector<cv::Point>> contours;
    vector<cv::Vec4i> hierarchy;
    cv::cvtColor(frame, frame_hsv, cv::COLOR_BGR2HSV);
    cv::inRange(frame_hsv, hsv_lower_bound, hsv_upper_bound, contour_image);
    cv::findContours(contour_image, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    detection_boxes->resize(contours.size());
    for (size_t index = 0; index < contours.size(); index++) {
        double area = cv::contourArea(contours.at(index));
        if (area < _min_contour_area || area > _max_contour_area) {
            contours.erase(contours.begin() + index);
            index--;
        }
    }
    for (size_t index = 0; index < contours.size(); index++) {
        detection_boxes->at(index) = cv::boundingRect(contours.at(index));
    }

    if (_pipeline_pub.getNumSubscribers() > 0) {
        cv::Mat result_image;
        cv::cvtColor(contour_image, result_image, cv::COLOR_GRAY2BGR);
        cv::drawContours(result_image, contours, -1, cv::Scalar(0, 255, 0), 1);
        if (!result_image.empty())
        {
            for (size_t index = 0; index < detection_boxes->size(); index++) {
                cv::rectangle(result_image, detection_boxes->at(index), cv::Scalar(255, 0, 0), 2);
            }

            std_msgs::Header header;
            header.frame_id = _camera_model.tfFrame();
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(header, "bgr8", result_image).toImageMsg();
            _pipeline_pub.publish(msg);
        }
    }
}

int LimelightTargetNode::run()
{
    ros::spin();
    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tj2_limelight");
    ros::NodeHandle nh;
    LimelightTargetNode node(&nh);
    return node.run();
}
