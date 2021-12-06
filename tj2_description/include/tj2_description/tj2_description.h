#pragma once

#include "ros/ros.h"
#include "ros/console.h"
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TwistStamped.h>
#include <tj2_tunnel/SwerveModule.h>

using namespace std;

class TJ2Description {
private:
    ros::NodeHandle nh;  // ROS node handle

    // Parameters
    int num_modules;
    string robot_type;

    // Publishers
    ros::Publisher wheel_joint_pub;
    vector<ros::Publisher>* twist_pubs;

    // Subscribers
    vector<ros::Subscriber>* wheel_subs;

    // Messages
    sensor_msgs::JointState wheel_joints_msg;
    vector<sensor_msgs::JointState>* twist_msgs;

    // Module callback
    void module_callback(const tj2_tunnel::SwerveModuleConstPtr& msg, int module_index);

    // Main loop methods
    void loop();
public:
    TJ2Description(ros::NodeHandle* nodehandle);
    int run();
};
