#include <tj2_description/tj2_description.h>


TJ2Description::TJ2Description(ros::NodeHandle* nodehandle):nh(*nodehandle)
{
    raw_joint_subs = new vector<ros::Subscriber>();
    
    joints_msg.header.frame_id = "base_link";
    joints_msg.name.push_back("left_outer_climber_joint");
    joints_msg.name.push_back("left_inner_climber_joint");
    joints_msg.name.push_back("right_outer_climber_joint");
    joints_msg.name.push_back("right_inner_climber_joint");
    joints_msg.name.push_back("left_outer_climber_hook_joint");
    joints_msg.name.push_back("left_inner_climber_hook_joint");
    joints_msg.name.push_back("right_outer_climber_hook_joint");
    joints_msg.name.push_back("right_inner_climber_hook_joint");
    joints_msg.name.push_back("intake_joint");
    joints_msg.name.push_back("turret_joint");
    joints_msg.name.push_back("camera_joint");

    for (int index = 0; index < joints_msg.name.size(); index++)
    {
        joints_msg.position.push_back(0.0);

        raw_joint_subs->push_back(
            nh.subscribe<std_msgs::Float64>(
                joints_msg.name.at(index), 50,
                boost::bind(&TJ2Description::joint_callback, this, _1, index)
            )
        );
    }
    
    joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 10);
    
    ROS_INFO("tj2_description is ready!");
}

void TJ2Description::joint_callback(const std_msgs::Float64ConstPtr& msg, int joint_index)
{
    joints_msg.position[joint_index] = msg->data;
    joints_msg.header.stamp = ros::Time::now();
}


void TJ2Description::loop()
{
    joint_pub.publish(joints_msg);
}

int TJ2Description::run()
{
    ros::Rate clock_rate(60);  // Hz

    int exit_code = 0;
    while (ros::ok())
    {
        // let ROS process any events
        ros::spinOnce();
        clock_rate.sleep();

        try {
            loop();
        }
        catch (exception& e) {
            ROS_ERROR_STREAM("Exception in main loop: " << e.what());
            exit_code = 1;
            break;
        }
    }
    return exit_code;
}
