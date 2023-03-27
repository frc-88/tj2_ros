#pragma once

#include "ntcore.h"

#include "ros/ros.h"
#include "ros/console.h"

#include "geometry_msgs/PoseStamped.h"

#include "tj2_interfaces/SwerveModule.h"
#include "tj2_interfaces/SwerveMotor.h"

#include "networktables/EntryListenerFlags.h"

#include "tj2_networktables.h"

using namespace std;


class ScorpionNetworkTables : public TJ2NetworkTables {
private:
    // Parameters
    string _tag_global_frame;

    // module entires
    NT_Entry _tag_global_entry;
    NT_Entry _nearest_cone_entry;

    // Members
    vector<double> _tag_global_pose;
    vector<double> _nearest_cone_pose;

    // Subscribers
    ros::Subscriber _nearest_cone_sub;

    // Other helpers
    void publish_module();
    void publish_tag_global_pose();

    void nearest_cone_callback(const geometry_msgs::PoseStampedConstPtr& msg);

    // Main loop methods
    void loop();

public:
    ScorpionNetworkTables(ros::NodeHandle* nodehandle);
    int run();
};
