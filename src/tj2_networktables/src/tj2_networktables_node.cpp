#include "diffyjr_networktables.h"
#include "scorpion_networktables.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tj2_networktables");
    ros::NodeHandle nh;

    std::string nt_interface_name;
    ros::param::param<std::string>("~interface", nt_interface_name, "diffyjr");

    TJ2NetworkTables* nt_interface = NULL;
    if (nt_interface_name.compare("diffyjr") == 0) {
        nt_interface = new DiffyJrNetworkTables(&nh);
    }
    else if (nt_interface_name.compare("scorpion") == 0) {
        nt_interface = new ScorpionNetworkTables(&nh);
    }
    else {
        ROS_ERROR("Failed to initialize networktables interface!! Check selected interface name.");
        return 1;
    }
    int err = nt_interface->run();

    return err;
}
