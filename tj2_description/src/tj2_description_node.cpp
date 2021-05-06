
#include "tj2_description/tj2_description.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tj2_description");
    ros::NodeHandle nh;

    TJ2Description broadcaster(&nh);
    int err = broadcaster.run();

    return err;
}
