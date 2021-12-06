#include <tj2_description/tj2_description.h>


TJ2Description::TJ2Description(ros::NodeHandle* nodehandle):nh(*nodehandle)
{
    ros::param::param<int>("~num_modules", num_modules, 4);

    wheel_subs = new vector<ros::Subscriber>();
    twist_pubs = new vector<ros::Publisher>();
    twist_msgs = new vector<geometry_msgs::TwistStamped>();

    wheel_joints_msg.header.frame_id = "base_link";
    for (int index = 0; index < num_modules; index++)
    {
        string wheel_name = to_string(index);
        wheel_joints_msg.name.push_back("base_link_to_wheel_" + wheel_name + "_joint");
        wheel_joints_msg.position.push_back(0.0);

        wheel_subs->push_back(
            nh.subscribe<tj2_tunnel::SwerveModule>(
                "swerve_modules/" + wheel_name, 50,
                boost::bind(&TJ2Description::module_callback, this, _1, index)
            )
        );

        geometry_msgs::TwistStamped msg;
        msg.header.frame_id = "swerve_wheel_" + wheel_name;
        twist_msgs->push_back(msg);
        twist_pubs->push_back(
            nh.advertise<geometry_msgs::TwistStamped>("swerve_module_twist/" + wheel_name, 10)
        );
    }
    
    wheel_joint_pub = nh.advertise<sensor_msgs::JointState>("wheel_joint_state", 10);
    
    ROS_INFO("tj2_description is ready!");
}

void TJ2Description::module_callback(const tj2_tunnel::SwerveModuleConstPtr& msg, int module_index)
{
    double azimuth = msg->azimuth_position;
    if (msg->wheel_velocity < 0) {
        azimuth -= M_PI;
    }
    
    wheel_joints_msg.position[module_index] = azimuth;
    wheel_joints_msg.header.stamp = ros::Time::now();

    geometry_msgs::TwistStamped twist_msg = twist_msgs->at(module_index);
    twist_msg.header.stamp = ros::Time::now();
    twist_msg.twist.linear.x = msg->wheel_velocity;
    twist_pubs->at(module_index).publish(twist_msg);
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
