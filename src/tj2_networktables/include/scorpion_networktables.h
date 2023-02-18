#pragma once

#include "ntcore.h"

#include "ros/ros.h"
#include "ros/console.h"

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

    // Members
    vector<double> _tag_global_pose;

    // Other helpers
    void publish_module();
    void publish_tag_global_pose();

    // Main loop methods
    void loop();

public:
    ScorpionNetworkTables(ros::NodeHandle* nodehandle);
    int run();
};
