#pragma once

#include "ntcore.h"

#include "ros/ros.h"
#include "ros/console.h"

#include "tj2_interfaces/SwerveModule.h"
#include "tj2_interfaces/SwerveMotor.h"

#include "networktables/EntryListenerFlags.h"

#include "tj2_networktables.h"

using namespace std;


class DiffyJrNetworkTables : public TJ2NetworkTables {
private:
    // Parameters
    string _tag_global_frame;

    // module entires
    NT_Entry _module_num_entry;
    NT_Entry _tag_global_entry;

    // Members
    int _num_modules;
    vector<double> _tag_global_pose;

    // Publishers
    ros::Publisher _modules_pub;

    // NT callbacks
    void module_num_callback(const nt::EntryNotification& event);

    // Timer callbacks

    // Other helpers
    void publish_module();
    void publish_tag_global_pose();

    // Main loop methods
    void loop();

public:
    DiffyJrNetworkTables(ros::NodeHandle* nodehandle);
    int run();
};
