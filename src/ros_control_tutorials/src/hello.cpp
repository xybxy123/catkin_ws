#include"ros_control_tutorials/hello.h"


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "hello_node");
    ros::NodeHandle nh;
    hello();
    ros::spin();
    ros::shutdown();
    return 0;
}
