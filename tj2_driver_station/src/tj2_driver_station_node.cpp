
#include "tj2_driver_station/tj2_driver_station.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tj2_driver_station");
    ros::NodeHandle nh;

    TJ2DriverStation broadcaster(&nh);
    int err = broadcaster.run();

    return err;
}
