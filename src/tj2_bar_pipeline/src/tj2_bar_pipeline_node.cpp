
#include "tj2_bar_pipeline/tj2_bar_pipeline.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tj2_bar_pipeline");
    ros::NodeHandle nh;

    TJ2BarPipeline broadcaster(&nh);
    int err = broadcaster.run();

    return err;
}
