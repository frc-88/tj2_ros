#include "tj2_imu_joint.h"

TJ2ImuJoint::TJ2ImuJoint(ros::NodeHandle* nodehandle) :
    nh(*nodehandle), _tf_listener(_tf_buffer)
{
    ros::param::param<double>("~publish_rate", _publish_rate, 30.0);
    ros::param::param<bool>("~navx_as_base", _navx_as_base, true);
    ros::param::param<string>("~base_parent_frame", _base_parent_frame, "base_link");
    ros::param::param<string>("~base_child_frame", _base_child_frame, "base_tilt_link");
    ros::param::param<string>("~base_imu_frame", _base_imu_frame, "imu");
    ros::param::param<string>("~camera_parent_frame", _camera_parent_frame, "camera_tilt_link");
    ros::param::param<string>("~camera_child_frame", _camera_child_frame, "camera_link");

    _static_imu_tf_set = false;
    
    _camera_imu_sub = nh.subscribe<sensor_msgs::Imu>("camera_imu", 10, &TJ2ImuJoint::camera_imu_callback, this);
    if (_navx_as_base) {
        _base_imu_sub = nh.subscribe<tj2_interfaces::NavX>("base_imu", 10, &TJ2ImuJoint::base_navx_callback, this);
    }
    else {
        _base_imu_sub = nh.subscribe<sensor_msgs::Imu>("base_imu", 10, &TJ2ImuJoint::base_imu_callback, this);
    }
}

void TJ2ImuJoint::camera_imu_callback(const sensor_msgs::ImuConstPtr& imu)
{
    tf2::convert(imu->orientation, _camera_imu_quat);
    _camera_imu_quat.normalize();

    tf2::Matrix3x3 m1(_base_to_base_tilt_quat.inverse());
    tf2::Matrix3x3 m2(_camera_imu_quat);
    
    m2 *= m1;

    double roll, pitch, yaw;
    m2.getRPY(roll, pitch, yaw);
    _camera_to_tilt_quat.setRPY(roll, pitch, 0.0);
    tf2::convert(_camera_to_tilt_quat, camera_quat_msg);

    // ROS_INFO("%0.4f, %0.4f, %0.4f", roll, pitch, yaw);
    // ROS_INFO("%0.4f, %0.4f, %0.4f, %0.4f", imu->orientation.x, imu->orientation.y, imu->orientation.z, imu->orientation.w);
    // ROS_INFO("%0.4f, %0.4f, %0.4f, %0.4f", _camera_to_tilt_quat.getX(), _camera_to_tilt_quat.getY(), _camera_to_tilt_quat.getZ(), _camera_to_tilt_quat.getW());
    // ROS_INFO("---");
}

void TJ2ImuJoint::base_imu_callback(const sensor_msgs::ImuConstPtr& imu)
{
    tf2::Quaternion quat;
    tf2::convert(imu->orientation, quat);
    base_callback(quat);
}

void TJ2ImuJoint::base_navx_callback(const tj2_interfaces::NavXConstPtr& imu)
{
    tf2::Quaternion quat;
    tf2::convert(imu->orientation, quat);
    base_callback(quat);
}

void TJ2ImuJoint::base_callback(tf2::Quaternion quat)
{
    if (!_static_imu_tf_set) {
        try {
            _static_imu_to_base_tf = _tf_buffer.lookupTransform(_base_imu_frame, _base_child_frame, ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            return;
        }
        tf2::convert(_static_imu_to_base_tf.transform.rotation, _static_imu_to_base_quat);
        _static_imu_to_base_mat.setRotation(_static_imu_to_base_quat);
        _static_imu_tf_set = true;
    }

    tf2::Matrix3x3 m1(quat);

    m1 *= _static_imu_to_base_mat;

    double roll, pitch, yaw;
    m1.getRPY(roll, pitch, yaw);
    _base_to_base_tilt_quat.setRPY(roll, pitch, yaw);

    // Yaw is set by odom so it's always zero here
    _base_to_base_tilt_quat.setRPY(roll, pitch, 0.0);
    tf2::convert(_base_to_base_tilt_quat, base_quat_msg);
}

void TJ2ImuJoint::publish_base_tf()
{
    if (!(std::isfinite(base_quat_msg.x) && std::isfinite(base_quat_msg.y) && std::isfinite(base_quat_msg.z) && std::isfinite(base_quat_msg.w))) {
        ROS_WARN_THROTTLE(1.0, "Base tilt quaternion contains NaNs or Infs!");
        return;
    }
    geometry_msgs::TransformStamped tf_stamped;
    tf_stamped.header.stamp = ros::Time::now();
    tf_stamped.header.frame_id = _base_parent_frame;
    tf_stamped.child_frame_id = _base_child_frame;
    tf_stamped.transform.translation.x = 0.0;
    tf_stamped.transform.translation.y = 0.0;
    tf_stamped.transform.translation.z = 0.0;
    tf_stamped.transform.rotation = base_quat_msg;

    _tf_broadcaster.sendTransform(tf_stamped);
}

void TJ2ImuJoint::publish_camera_tf()
{
    if (!(std::isfinite(camera_quat_msg.x) && std::isfinite(camera_quat_msg.y) && std::isfinite(camera_quat_msg.z) && std::isfinite(camera_quat_msg.w))) {
        ROS_WARN_THROTTLE(1.0, "Camera tilt quaternion contains NaNs or Infs!");
        return;
    }
    geometry_msgs::TransformStamped tf_stamped;
    tf_stamped.header.stamp = ros::Time::now();
    tf_stamped.header.frame_id = _camera_parent_frame;
    tf_stamped.child_frame_id = _camera_child_frame;
    tf_stamped.transform.translation.x = 0.0;
    tf_stamped.transform.translation.y = 0.0;
    tf_stamped.transform.translation.z = 0.0;
    tf_stamped.transform.rotation = camera_quat_msg;

    _tf_broadcaster.sendTransform(tf_stamped);
}

int TJ2ImuJoint::run()
{
    ros::Rate clock_rate(_publish_rate);  // Hz
    while (ros::ok())
    {
        clock_rate.sleep();
        publish_base_tf();
        publish_camera_tf();
        ros::spinOnce();
    }
    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tj2_imu_joint");
    ros::NodeHandle nh;
    TJ2ImuJoint node(&nh);
    return node.run();
}
