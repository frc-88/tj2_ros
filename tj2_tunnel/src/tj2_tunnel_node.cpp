
#include "tj2_tunnel/tj2_tunnel.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tj2_tunnel");
    ros::NodeHandle nh;

    TJ2Tunnel broadcaster(&nh);
    int err = broadcaster.run();

    return err;
}
