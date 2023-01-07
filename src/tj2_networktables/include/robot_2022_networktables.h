#pragma once

#include "ntcore.h"

#include "ros/ros.h"
#include "ros/console.h"

#include "tj2_interfaces/SwerveModule.h"
#include "tj2_interfaces/SwerveMotor.h"

#include "tj2_interfaces/Shooter.h"
#include "tj2_interfaces/Hood.h"

// #include "tj2_target/TargetConfig.h"

#include "networktables/EntryListenerFlags.h"

#include "tj2_networktables.h"

using namespace std;

class Robot2022NetworkTables : public TJ2NetworkTables {
private:
    // Parameters

    // shooter entries
    NT_Entry _hood_state_entry;
    NT_Entry _hood_update_entry;
    NT_Entry _shoot_counter_entry;
    NT_Entry _shoot_speed_entry;
    NT_Entry _shoot_angle_entry;
    NT_Entry _shoot_distance_entry;

    // Reset localization entries
    NT_Entry _reset_to_limelight_entry;

    // Target config
    NT_Entry _enable_shot_correction_entry;
    NT_Entry _enable_moving_shot_probability_entry;
    NT_Entry _enable_stationary_shot_probability_entry;
    NT_Entry _enable_limelight_fine_tuning_entry;
    NT_Entry _enable_marauding_entry;
    NT_Entry _enable_reset_to_limelight_entry;
    NT_Entry _target_config_update_entry;

    // module entires
    NT_Entry _module_num_entry;

    // Members
    int _num_modules;

    // Publishers
    ros::Publisher _hood_pub;
    ros::Publisher _shooter_pub;
    ros::Publisher _reset_to_limelight_pub;
    // ros::Publisher _target_config_pub;
    ros::Publisher _modules_pub;

    // NT callbacks
    void reset_to_limelight_callback(const nt::EntryNotification& event);
    void target_config_callback(const nt::EntryNotification& event);
    void module_num_callback(const nt::EntryNotification& event);
    void hood_state_callback(const nt::EntryNotification& event);
    void shooter_callback(const nt::EntryNotification& event);

    // Other helpers
    void publish_module();

    // Main loop methods
    void loop();

public:
    Robot2022NetworkTables(ros::NodeHandle* nodehandle);
    int run();
};