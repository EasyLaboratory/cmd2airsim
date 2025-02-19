#include "easysim_ros_wrapper.h"

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "easysim_ros_wrapper_node");

    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~"); 

    EasySimRosWrapper easysim_ros_wrapper(nh,nh_private);

    easysim_ros_wrapper.start();

    return 0;
}
