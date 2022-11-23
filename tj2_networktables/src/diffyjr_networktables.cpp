#include "diffyjr_networktables.h"

DiffyJrNetworkTables::DiffyJrNetworkTables(ros::NodeHandle* nodehandle) :
    TJ2NetworkTables(nodehandle)
{
    _modules_pub = nh.advertise<tj2_interfaces::SwerveModule>("swerve_modules", 10);

    _num_modules = 0;

    _module_num_entry = nt::GetEntry(_nt, _base_key + "modules/num");
    nt::AddEntryListener(_module_num_entry, boost::bind(&DiffyJrNetworkTables::module_num_callback, this, _1), nt::EntryListenerFlags::kNew | nt::EntryListenerFlags::kUpdate);

    ROS_INFO("diffyjr_networktables init complete");
}

// ---
// NT callbacks
// ---

void DiffyJrNetworkTables::module_num_callback(const nt::EntryNotification& event)
{
    _num_modules = (int)get_double(_module_num_entry, 0);
}

// ---
// Other helpers
// ---

void DiffyJrNetworkTables::publish_module()
{
    for (int index = 0; index < _num_modules; index++) {
        string module_index = std::to_string(index);
        tj2_interfaces::SwerveModule module_msg;
        tj2_interfaces::SwerveMotor lo_motor_msg;
        tj2_interfaces::SwerveMotor hi_motor_msg;
        module_msg.module_index = module_index;
        module_msg.wheel_velocity = get_double(get_entry("modules/" + module_index + "/wheel_velocity"), 0.0);
        module_msg.azimuth_velocity = get_double(get_entry("modules/" + module_index + "/azimuth_velocity"), 0.0);
        module_msg.azimuth_position = get_double(get_entry("modules/" + module_index + "/azimuth"), 0.0);
        module_msg.wheel_velocity_ref = get_double(get_entry("modules/" + module_index + "/wheel_velocity_ref"), 0.0);
        module_msg.azimuth_velocity_ref = get_double(get_entry("modules/" + module_index + "/azimuth_velocity_ref"), 0.0);
        module_msg.azimuth_position_ref = get_double(get_entry("modules/" + module_index + "/azimuth_ref"), 0.0);
        hi_motor_msg.voltage = get_double(get_entry("modules/" + module_index + "/hi_voltage"), 0.0);
        hi_motor_msg.voltage_ref = get_double(get_entry("modules/" + module_index + "/hi_voltage_ref"), 0.0);
        hi_motor_msg.velocity = get_double(get_entry("modules/" + module_index + "/hi_velocity"), 0.0);
        lo_motor_msg.voltage = get_double(get_entry("modules/" + module_index + "/lo_voltage"), 0.0);
        lo_motor_msg.voltage_ref = get_double(get_entry("modules/" + module_index + "/lo_voltage_ref"), 0.0);
        lo_motor_msg.velocity = get_double(get_entry("modules/" + module_index + "/lo_velocity"), 0.0);
        module_msg.motor_lo_0 = lo_motor_msg;
        module_msg.motor_hi_1 = hi_motor_msg;
        _modules_pub.publish(module_msg);
    }
}

// ---
// Main loop methods
// ---

void DiffyJrNetworkTables::loop()
{
    TJ2NetworkTables::loop();
    publish_module();
}
