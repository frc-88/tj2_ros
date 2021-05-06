#pragma once

#include "ros/ros.h"
#include "ros/console.h"
#include <sensor_msgs/JointState.h>
#include <tj2_networktables/SwerveModule.h>

using namespace std;

class TJ2Description {
private:
    ros::NodeHandle nh;  // ROS node handle

    // Parameters
    int num_modules;

    // Publishers
    ros::Publisher wheel_joint_pub;

    // Subscribers
    vector<ros::Subscriber>* wheel_subs;

    // Joint messages
    sensor_msgs::JointState wheel_joints_msg;

    // Module callback
    void module_callback(const tj2_networktables::SwerveModuleConstPtr& msg, int module_index);

    // Main loop methods
    void loop();
public:
    TJ2Description(ros::NodeHandle* nodehandle);
    int run();
};
