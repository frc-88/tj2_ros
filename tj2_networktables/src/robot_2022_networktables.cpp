#include "robot_2022_networktables.h"

Robot2022NetworkTables::Robot2022NetworkTables(ros::NodeHandle* nodehandle) :
    TJ2NetworkTables(nodehandle)
{
    _hood_pub = nh.advertise<tj2_interfaces::Hood>("hood", 10);
    _shooter_pub = nh.advertise<tj2_interfaces::Shooter>("shooter", 10);
    _reset_to_limelight_pub = nh.advertise<std_msgs::Float64>("reset_to_limelight", 10);
    _target_config_pub = nh.advertise<tj2_target::TargetConfig>("target_config", 10);

    _modules_pub = nh.advertise<tj2_interfaces::SwerveModule>("swerve_modules", 10);

    _num_modules = 0;

    _hood_state_entry = nt::GetEntry(_nt, _base_key + "hood/state");
    _hood_update_entry = nt::GetEntry(_nt, _base_key + "hood/update");
    nt::AddEntryListener(_hood_update_entry, boost::bind(&Robot2022NetworkTables::hood_state_callback, this, _1), nt::EntryListenerFlags::kNew | nt::EntryListenerFlags::kUpdate);

    _shoot_counter_entry = nt::GetEntry(_nt, _base_key + "shooter/counter");
    _shoot_speed_entry = nt::GetEntry(_nt, _base_key + "shooter/speed");
    _shoot_angle_entry = nt::GetEntry(_nt, _base_key + "shooter/angle");
    _shoot_distance_entry = nt::GetEntry(_nt, _base_key + "shooter/distance");
    nt::AddEntryListener(_shoot_counter_entry, boost::bind(&Robot2022NetworkTables::shooter_callback, this, _1), nt::EntryListenerFlags::kNew | nt::EntryListenerFlags::kUpdate);

    _reset_to_limelight_entry = nt::GetEntry(_nt, _base_key + "resetToLimelight/update");
    nt::AddEntryListener(_reset_to_limelight_entry, boost::bind(&Robot2022NetworkTables::reset_to_limelight_callback, this, _1), nt::EntryListenerFlags::kNew | nt::EntryListenerFlags::kUpdate);

    _enable_shot_correction_entry = nt::GetEntry(_nt, _base_key + "target_config/enable_shot_correction");
    _enable_moving_shot_probability_entry = nt::GetEntry(_nt, _base_key + "target_config/enable_moving_shot_probability");
    _enable_stationary_shot_probability_entry = nt::GetEntry(_nt, _base_key + "target_config/enable_stationary_shot_probability");
    _enable_limelight_fine_tuning_entry = nt::GetEntry(_nt, _base_key + "target_config/enable_limelight_fine_tuning");
    _enable_marauding_entry = nt::GetEntry(_nt, _base_key + "target_config/enable_marauding");
    _enable_reset_to_limelight_entry = nt::GetEntry(_nt, _base_key + "target_config/enable_reset_to_limelight");
    _target_config_update_entry = nt::GetEntry(_nt, _base_key + "target_config/update");
    nt::AddEntryListener(_target_config_update_entry, boost::bind(&Robot2022NetworkTables::target_config_callback, this, _1), nt::EntryListenerFlags::kNew | nt::EntryListenerFlags::kUpdate);

    _module_num_entry = nt::GetEntry(_nt, _base_key + "modules/num");
    nt::AddEntryListener(_module_num_entry, boost::bind(&Robot2022NetworkTables::module_num_callback, this, _1), nt::EntryListenerFlags::kNew | nt::EntryListenerFlags::kUpdate);

    ROS_INFO("robot_2022_networktables init complete");
}

// ---
// NT callbacks
// ---

void Robot2022NetworkTables::module_num_callback(const nt::EntryNotification& event)
{
    _num_modules = (int)get_double(_module_num_entry, 0);
}

void Robot2022NetworkTables::hood_state_callback(const nt::EntryNotification& event)
{
    double hood_angle = get_double(_hood_state_entry, false);
    tj2_interfaces::Hood msg;
    msg.angle = hood_angle;
    _hood_pub.publish(msg);
}

void Robot2022NetworkTables::shooter_callback(const nt::EntryNotification& event)
{
    int counter = (int)get_double(_shoot_counter_entry, 0.0);
    double speed = get_double(_shoot_speed_entry, 0.0);
    double angle = get_double(_shoot_angle_entry, 0.0);
    double distance = get_double(_shoot_distance_entry, 0.0);
    tj2_interfaces::Shooter msg;
    msg.counter = counter;
    msg.speed = speed;
    msg.angle = angle;
    msg.distance = distance;
    
    msg.header.stamp = ros::Time::now();
    _shooter_pub.publish(msg);
}

void Robot2022NetworkTables::reset_to_limelight_callback(const nt::EntryNotification& event)
{
    std_msgs::Float64 msg;
    msg.data = get_double(_reset_to_limelight_entry, 0.0);
    _reset_to_limelight_pub.publish(msg);
}


void Robot2022NetworkTables::target_config_callback(const nt::EntryNotification& event)
{
    tj2_target::TargetConfig msg;
    msg.enable_shot_correction = (int)get_double(_enable_shot_correction_entry, -1.0);
    msg.enable_moving_shot_probability = (int)get_double(_enable_moving_shot_probability_entry, -1.0);
    msg.enable_stationary_shot_probability = (int)get_double(_enable_stationary_shot_probability_entry, -1.0);
    msg.enable_limelight_fine_tuning = (int)get_double(_enable_limelight_fine_tuning_entry, -1.0);
    msg.enable_marauding = (int)get_double(_enable_marauding_entry, -1.0);
    msg.enable_reset_to_limelight = (int)get_double(_enable_reset_to_limelight_entry, -1.0);
    _target_config_pub.publish(msg);
}


// ---
// Other helpers
// ---

void Robot2022NetworkTables::publish_module()
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

void Robot2022NetworkTables::loop()
{
    TJ2NetworkTables::loop();
    publish_module();
}
