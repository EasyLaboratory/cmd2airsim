#include "easysim_ros_wrapper.h"

EasySimRosWrapper::EasySimRosWrapper(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private)
{   


    getParams();
    std::cout<<"ip port"<<ip_address<<" "<<port<<std::endl;

    easytrack_client_ = std::make_shared<msr::airlib::easyTrackRpcClient>(ip_address, port);
    odom_pub_ = nh_private_.advertise<nav_msgs::Odometry>("player_odom", 10);
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
    // Retrieve player position
    Eigen::Vector3f player_position = easytrack_client_->getPlayerPosition(1) / 100.0;

    // Convert to ENU frame if needed
    if (coordinate_frame == "world_enu") {
        player_position = worldToENU(player_position);
    }

    // Get the current time
    ros::Time current_time = ros::Time::now();
    Eigen::Vector3f linear_velocity(0.0, 0.0, 0.0);

    if (!first_frame_) {
        // Calculate time difference
        double dt = (current_time - previous_time_).toSec();

        // Define a threshold for the maximum allowed time difference
        double max_time_difference = 1.0; // 1 second, adjust as needed

        if (dt > 0 && dt < max_time_difference) {
            // Calculate velocity if the time difference is within the acceptable range
            linear_velocity = (player_position - previous_position_) / dt;
        } else {
            // If the time difference is too large, treat this as a new frame and reset velocity
            linear_velocity = Eigen::Vector3f(0.0, 0.0, 0.0);
            // first_frame_=true;
        }
    } else {
        // Mark that the first frame has been processed
        first_frame_ = false;
    }




    // Update the previous position and time for the next callback
    previous_position_ = player_position;
    previous_time_ = current_time;

    // Create and publish the Odometry message
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = current_time;
    odom_msg.header.frame_id = coordinate_frame;
    odom_msg.pose.pose.position.x = player_position.x();
    odom_msg.pose.pose.position.y = player_position.y();
    odom_msg.pose.pose.position.z = player_position.z();
    
    odom_msg.twist.twist.linear.x = linear_velocity.x();
    odom_msg.twist.twist.linear.y = linear_velocity.y();
    odom_msg.twist.twist.linear.z = linear_velocity.z();

    odom_pub_.publish(odom_msg);
}


Eigen::Vector3f EasySimRosWrapper::worldToENU(const Eigen::Vector3f& world_position)
{
    return Eigen::Vector3f(world_position.y(),world_position.x(), world_position.z());

}