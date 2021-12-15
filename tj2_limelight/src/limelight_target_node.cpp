#include "limelight_target_node.h"

LimelightTargetNode::LimelightTargetNode(ros::NodeHandle* nodehandle) :
    nh(*nodehandle),
    _image_transport(nh),
    _tf_listener(_tf_buffer)
{
    ros::param::param<double>("~text_marker_size", _text_marker_size, 0.25);
    ros::param::param<string>("~target_frame", _target_frame, "base_link");
    ros::param::param<double>("~marker_persistance_s", _marker_persistance_s, 0.1);
    _marker_persistance = ros::Duration(_marker_persistance_s);

    _marker_pub = nh.advertise<visualization_msgs::MarkerArray>("target_marker", 10);
    _target_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("target_pose", 10);

    _depth_sub.subscribe(nh, "depth/image_raw", 10);
    _depth_info_sub.subscribe(nh, "depth/camera_info", 10);
    _target_sub.subscribe(nh, "/limelight/target", 10);

    sync.reset(new Sync(ApproxSyncPolicy(10), _depth_sub, _depth_info_sub, _target_sub));
    sync->registerCallback(boost::bind(&LimelightTargetNode::target_callback, this, _1, _2, _3));
}

void LimelightTargetNode::target_callback(const ImageConstPtr& depth_image, const CameraInfoConstPtr& depth_info, const tj2_limelight::LimelightTargetConstPtr& target)
{
    if (!target->tv) {
        return;
    }

    // Extract info from messages

    _camera_model.fromCameraInfo(depth_info);

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(depth_image);  // encoding: passthrough
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat depth_cv_image;
    if (depth_image->width == 0 || depth_image->height == 0) {
        ROS_ERROR("Depth image has a zero width dimension!");
        return;
    }
    depth_cv_image = cv_ptr->image;

    // Project limelight target onto depth image
    cv::Point2d target_point;
    target_point.x = target->tx;
    target_point.y = target->ty;
    cv::Point3d ray = _camera_model.projectPixelTo3dRay(target_point);

    double z_dist = get_target_z(depth_cv_image, target);

    cv::Point3d center;
    center.x = ray.x * z_dist;
    center.y = ray.y * z_dist;
    center.z = z_dist;

    ROS_DEBUG("center, X: %0.3f, Y: %0.3f, Z: %0.3f", center.x, center.y, center.z);

    cv::Point2d edge_point;
    edge_point.x = target->tx + target->thor;
    edge_point.y = target->ty + target->tvert;
    ray = _camera_model.projectPixelTo3dRay(edge_point);

    cv::Point3d edge;
    edge.x = ray.x * z_dist;
    edge.y = ray.y * z_dist;
    edge.z = z_dist;
    ROS_DEBUG("edge,   X: %0.3f, Y: %0.3f, Z: %0.3f", edge.x, edge.y, edge.z);

    geometry_msgs::PoseStamped object_pose;

    object_pose.header.frame_id = _camera_model.tfFrame();
    object_pose.header.stamp = depth_image->header.stamp;
    object_pose.pose.position.x = center.x;
    object_pose.pose.position.y = center.y;
    object_pose.pose.position.z = center.z;
    object_pose.pose.orientation.x = 0.0;
    object_pose.pose.orientation.y = 0.0;
    object_pose.pose.orientation.z = 0.0;
    object_pose.pose.orientation.w = 1.0;

    // Publish to topics
    publish_target_pose(object_pose, depth_info, center);
    publish_markers(object_pose, center, edge);
}

void LimelightTargetNode::publish_target_pose(geometry_msgs::PoseStamped object_pose, const CameraInfoConstPtr depth_info, cv::Point3d center)
{
    
    geometry_msgs::TransformStamped transform_camera_to_target;

    try {
        transform_camera_to_target = _tf_buffer.lookupTransform(
            _target_frame, depth_info->header.frame_id,
            depth_info->header.stamp, ros::Duration(1.0)
        );
        tf2::doTransform(object_pose, object_pose, transform_camera_to_target);
        // obj_desc now contains the object position in the target frame

        object_pose.header.frame_id = _target_frame;

        _target_pose_pub.publish(object_pose);
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
}

void LimelightTargetNode::publish_markers(geometry_msgs::PoseStamped object_pose, cv::Point3d center, cv::Point3d edge)
{
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker rect_marker = make_marker(object_pose, center, edge);
    visualization_msgs::Marker text_marker = make_marker(object_pose, center, edge);

    rect_marker.type = visualization_msgs::Marker::CUBE;
    rect_marker.ns = "cube_" + rect_marker.ns;

    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.text = text_marker.ns;
    text_marker.ns = "text_" + text_marker.ns;
    text_marker.pose.position.z += edge.y - center.y;
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

visualization_msgs::Marker LimelightTargetNode::make_marker(geometry_msgs::PoseStamped object_pose, cv::Point3d center, cv::Point3d edge)
{
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = object_pose.pose;
    marker.header = object_pose.header;
    marker.lifetime = _marker_persistance;
    marker.ns = "limelight_target";
    marker.id = 0;

    geometry_msgs::Vector3 scale_vector;
    scale_vector.x = edge.x - center.x;
    scale_vector.y = edge.y - center.y;
    scale_vector.z = 0.005;
    marker.scale = scale_vector;
    marker.color = std_msgs::ColorRGBA();
    marker.color.r = 1.0;
    marker.color.a = 0.5;

    return marker;
}

double LimelightTargetNode::get_target_z(cv::Mat depth_cv_image, tj2_limelight::LimelightTargetConstPtr target)
{
    ROS_DEBUG("Depth image size: w=%d, h=%d", depth_cv_image.rows, depth_cv_image.cols);
    cv::Mat rectangle_mask = cv::Mat::zeros(depth_cv_image.rows, depth_cv_image.cols, CV_8UC1);
    cv::Point pt1(target->tx - target->thor / 2, target->ty - target->tvert / 2);
    cv::Point pt2(target->tx + target->thor / 2, target->ty + target->tvert / 2);
    cv::rectangle(rectangle_mask, pt1, pt2, cv::Scalar(255, 255, 255), cv::FILLED);
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
