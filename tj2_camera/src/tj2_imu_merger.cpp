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
}

void TJ2ImuMerger::_accel_callback(const sensor_msgs::ImuConstPtr& accel)
{
    _imu_msg.header = accel->header;
    _imu_msg.orientation.x = 0.0;
    _imu_msg.orientation.y = 0.0;
    _imu_msg.orientation.z = 0.0;
    _imu_msg.orientation.w = 1.0;
    
    _imu_msg.linear_acceleration = accel->linear_acceleration;
    _imu_msg.linear_acceleration_covariance = accel->linear_acceleration_covariance;
}

void TJ2ImuMerger::_gyro_callback(const sensor_msgs::ImuConstPtr& gyro)
{
    _imu_msg.header = gyro->header;
    _imu_msg.orientation.x = 0.0;
    _imu_msg.orientation.y = 0.0;
    _imu_msg.orientation.z = 0.0;
    _imu_msg.orientation.w = 1.0;
    
    _imu_msg.angular_velocity = gyro->angular_velocity;
    _imu_msg.angular_velocity_covariance = gyro->angular_velocity_covariance;
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

        _imu_msg.header.frame_id = _imu_base_frame;
        _sync_imu_pub.publish(_imu_msg);
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
