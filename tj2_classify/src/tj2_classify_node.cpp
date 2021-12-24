
#include "tj2_classify/tj2_classify.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tj2_classify");
    ros::NodeHandle nh;

    TJ2Classify broadcaster(&nh);
    int err = broadcaster.run();

    return err;
}
