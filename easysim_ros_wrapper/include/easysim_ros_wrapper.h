#ifndef EASYSIM_ROS_WRAPPER_HPP
#define EASYSIM_ROS_WRAPPER_HPP

#include "easytrack/easyTrackRpcClient.hpp"
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <memory>
#include "nav_msgs/Odometry.h"
#include <random>

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

    ros::Publisher odom_pub_;
    ros::Publisher position_pub_;
    ros::Timer timer_;
    ros::NodeHandle nh_private_;
    ros::NodeHandle nh_;
        // Retrieve parameters from the ROS parameter server
    std::string ip_address;
    uint16_t port;
    double timer_duration;
    std::string coordinate_frame;

    Eigen::Vector3f previous_position_;
    ros::Time previous_time_;
    bool first_frame_=true;

    bool use_noise_;

    float noise_x_std_dev=5.0;
    float noise_y_std_dev=5.0;
    float noise_z_std_dev=5.0;

    std::default_random_engine random_engine_;

    std::normal_distribution<float>* normal_dist_x_;
    std::normal_distribution<float>* normal_dist_y_;
    std::normal_distribution<float>* normal_dist_z_;

};


#endif // EASYSIM_ROS_WRAPPER_HPP
