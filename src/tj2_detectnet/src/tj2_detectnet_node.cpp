
#include "tj2_detectnet/tj2_detectnet.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tj2_detectnet");
    ros::NodeHandle nh;

    TJ2DetectNet broadcaster(&nh);
    int err = broadcaster.run();

    return err;
}
