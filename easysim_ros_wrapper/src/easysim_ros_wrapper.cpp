#include "easysim_ros_wrapper.h"

EasySimRosWrapper::EasySimRosWrapper(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private)
{   


    getParams();
    std::cout<<"ip port"<<ip_address<<" "<<port<<std::endl;

    easytrack_client_ = std::make_shared<msr::airlib::easyTrackRpcClient>(ip_address, port);

    position_pub_ = nh_private_.advertise<geometry_msgs::PointStamped>("player_position", 10);
    timer_ = nh_private_.createTimer(ros::Duration(timer_duration), &EasySimRosWrapper::timerCallback, this);

}

void EasySimRosWrapper::getParams(){

    nh_private_.param<std::string>("ip_address", ip_address, "127.0.0.1");
    nh_private_.param("timer_duration", timer_duration, 0.01);  // Default is 0.01 seconds
    nh_private_.param<std::string>("coordinate_frame", coordinate_frame, "world_enu"); // Default is "world"

    int temp_port;
    nh_private_.param("port", temp_port);

    std::cout<<"temp_port"<<temp_port<<std::endl;

    if (temp_port >= 0 && temp_port <= std::numeric_limits<uint16_t>::max()) {
        port = static_cast<uint16_t>(temp_port);
        
        }
    else{
        ROS_WARN("Parameter 'port' not set, using default value");
        port = 50502; // 默认值
        ROS_INFO("Port value: %u", port);
    }

}

void EasySimRosWrapper::start()
{
    ros::spin();
}


void EasySimRosWrapper::timerCallback(const ros::TimerEvent& event)
{
    try {
        Eigen::Vector3f player_position = easytrack_client_->getPlayerPosition(1) / 100.0;

        // Convert to ENU frame if needed
        if (coordinate_frame == "world_enu") {
            player_position = worldToENU(player_position);
        }
    
        geometry_msgs::PointStamped msg;
        msg.header.frame_id = coordinate_frame;
        msg.header.stamp = ros::Time::now();
        msg.point.x = player_position.x();
        msg.point.y = player_position.y();
        msg.point.z = player_position.z();

        position_pub_.publish(msg);

    } catch (const std::exception& e) {

        ROS_ERROR("Error: %s", e.what());

    }
}

Eigen::Vector3f EasySimRosWrapper::worldToENU(const Eigen::Vector3f& world_position)
{
    return Eigen::Vector3f(world_position.y(),world_position.x(), world_position.z());

}