#include <tj2_description/tj2_description.h>


TJ2Description::TJ2Description(ros::NodeHandle* nodehandle):nh(*nodehandle)
{
    ros::param::param<int>("~num_modules", num_modules, 4);

    wheel_joints_msg.header.frame_id = "base_link";
    for (int index = 0; index < num_modules; index++)
    {
        string wheel_name = to_string(index + 1);
        wheel_joints_msg.name.push_back("base_link_to_wheel_" + wheel_name + "_joint");
        wheel_joints_msg.position.push_back(0.0);

        wheel_subs.push_back(
            nh.subscribe<tj2_networktables::SwerveModule>(
                "swerve_modules/" + wheel_name, 50,
                boost::bind(&TJ2Description::module_callback, *this, _1, index)
            )
        );
    }

    wheel_joint_pub = nh.advertise<sensor_msgs::JointState>("wheel_joint_state", 10);
}

void TJ2Description::module_callback(const tj2_networktables::SwerveModuleConstPtr& msg, int module_index)
{
    ros::Time now = ros::Time::now();

    wheel_joints_msg.position[module_index] = msg->azimuth;
    wheel_joints_msg.header.stamp = now;
}


void TJ2Description::loop()
{
    wheel_joint_pub.publish(wheel_joints_msg);
}

int TJ2Description::run()
{
    ros::Rate clock_rate(50);  // Hz

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
