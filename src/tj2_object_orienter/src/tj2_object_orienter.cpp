#include "tj2_object_orienter.h"


TJ2ObjectOrienter::TJ2ObjectOrienter(ros::NodeHandle* nodehandle) :
    nh(*nodehandle), _tf_listener(_tf_buffer)
{

}

int TJ2ObjectOrienter::run()
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tj2_object_orienter");
    ros::NodeHandle nh;
    TJ2ObjectOrienter node(&nh);
    int err = node.run();
    return err;
}
