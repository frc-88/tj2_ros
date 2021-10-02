
#include "tj2_networktables/tj2_networktables.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tj2_networktables");
    ros::NodeHandle nh;

    TJ2NetworkTables broadcaster(&nh);
    int err = broadcaster.run();

    return err;
}
