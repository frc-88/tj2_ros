#pragma once

#include "ros/ros.h"

#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>

#include "tf/transform_listener.h"


using namespace std;


class TJ2ImuMerger
{
public:
    TJ2ImuMerger(ros::NodeHandle* nodehandle);
    int run();
private:
    ros::NodeHandle nh;  // ROS node handle

    // Parameters
    string _imu_base_frame;
    double _publish_rate;
    double _sync_time_threshold;

    // Members
    sensor_msgs::Imu _imu_msg;
    std_msgs::Float64 _joint_msg;

    // Publishers
    ros::Publisher _sync_imu_pub;
    ros::Publisher _camera_joint_pub;

    // Subscribers
    ros::Subscriber _accel_imu_sub;
    ros::Subscriber _gyro_imu_sub;
    ros::Subscriber _imu_filtered_sub;

    // Sub callbacks
    void _accel_callback(const sensor_msgs::ImuConstPtr& accel);
    void _gyro_callback(const sensor_msgs::ImuConstPtr& gyro);
    void _imu_filtered_callback(const sensor_msgs::ImuConstPtr& imu);
};
