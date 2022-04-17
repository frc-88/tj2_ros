#include "tj2_imu_merger.h"

TJ2ImuMerger::TJ2ImuMerger(ros::NodeHandle* nodehandle) :
    nh(*nodehandle)
{
    ros::param::param<string>("~imu_base_frame", _imu_base_frame, "imu");
    ros::param::param<double>("~publish_rate", _publish_rate, 10.0);
    ros::param::param<double>("~sync_time_threshold", _sync_time_threshold, 0.25);
    
    _accel_imu_sub = nh.subscribe<sensor_msgs::Imu>("accel", 10, &TJ2ImuMerger::_accel_callback, this);
    _gyro_imu_sub = nh.subscribe<sensor_msgs::Imu>("gyro", 10, &TJ2ImuMerger::_gyro_callback, this);
    _imu_filtered_sub = nh.subscribe<sensor_msgs::Imu>("imu/filtered", 10, &TJ2ImuMerger::_imu_filtered_callback, this);
    
    _sync_imu_pub = nh.advertise<sensor_msgs::Imu>("imu", 10);
    _camera_joint_pub = nh.advertise<std_msgs::Float64>("joint", 10);

    _imu_msg.header.frame_id = _imu_base_frame;
}

void TJ2ImuMerger::_accel_callback(const sensor_msgs::ImuConstPtr& accel)
{
    _imu_msg.header.stamp = accel->header.stamp;
    _imu_msg.orientation.x = 0.0;
    _imu_msg.orientation.y = 0.0;
    _imu_msg.orientation.z = 0.0;
    _imu_msg.orientation.w = 1.0;
    
    _imu_msg.linear_acceleration.x = accel->linear_acceleration.x;
    _imu_msg.linear_acceleration.y = accel->linear_acceleration.y;
    _imu_msg.linear_acceleration.z = accel->linear_acceleration.z;
    for (size_t index = 0; index < accel->linear_acceleration_covariance.size(); index++) {
        _imu_msg.linear_acceleration_covariance.at(index) = accel->linear_acceleration_covariance.at(index);
    }
}

void TJ2ImuMerger::_gyro_callback(const sensor_msgs::ImuConstPtr& gyro)
{
    _imu_msg.header.stamp = gyro->header.stamp;
    _imu_msg.orientation.x = 0.0;
    _imu_msg.orientation.y = 0.0;
    _imu_msg.orientation.z = 0.0;
    _imu_msg.orientation.w = 1.0;
    
    _imu_msg.angular_velocity.x = gyro->angular_velocity.x;
    _imu_msg.angular_velocity.y = gyro->angular_velocity.y;
    _imu_msg.angular_velocity.z = gyro->angular_velocity.z;
    for (size_t index = 0; index < gyro->angular_velocity_covariance.size(); index++) {
        _imu_msg.angular_velocity_covariance.at(index) = gyro->angular_velocity_covariance.at(index);
    }
}

void TJ2ImuMerger::_imu_filtered_callback(const sensor_msgs::ImuConstPtr& imu)
{
    tf::Quaternion q(
        imu->orientation.x,
        imu->orientation.y,
        imu->orientation.z,
        imu->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    roll += M_PI / 2.0;
    roll *= -1.0;
    _joint_msg.data = roll;
}


int TJ2ImuMerger::run()
{
    ros::Rate clock_rate(_publish_rate);  // Hz
    while (ros::ok())
    {
        clock_rate.sleep();
        _camera_joint_pub.publish(_joint_msg);
        _sync_imu_pub.publish(_imu_msg);
        ros::spinOnce();
    }
    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tj2_imu_merger");
    ros::NodeHandle nh;
    TJ2ImuMerger node(&nh);
    return node.run();
}
