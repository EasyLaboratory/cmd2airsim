#ifndef EASYSIM_ROS_WRAPPER_HPP
#define EASYSIM_ROS_WRAPPER_HPP

#include "easytrack/easyTrackRpcClient.hpp"
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <memory>

class EasySimRosWrapper
{

    
public:
    EasySimRosWrapper(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    void start();

private:
    void timerCallback(const ros::TimerEvent& event);
    void getParams();

    Eigen::Vector3f worldToENU(const Eigen::Vector3f& world_position);


    std::shared_ptr<msr::airlib::easyTrackRpcClient> easytrack_client_;


    ros::Publisher position_pub_;
    ros::Timer timer_;
    ros::NodeHandle nh_private_;
    ros::NodeHandle nh_;
        // Retrieve parameters from the ROS parameter server
    std::string ip_address;
    uint16_t port;
    double timer_duration;
    std::string coordinate_frame;

};


#endif // EASYSIM_ROS_WRAPPER_HPP
