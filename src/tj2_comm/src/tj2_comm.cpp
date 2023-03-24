
#include "tj2_comm.h"

TJ2Comm::TJ2Comm(ros::NodeHandle* nodehandle) :
    nh(*nodehandle), _tf_listener(_tf_buffer)
{

}

int TJ2Comm::run()
{
    ros::spin();
    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tj2_comm");
    ros::NodeHandle nh;
    TJ2Comm node(&nh);
    return node.run();
}
