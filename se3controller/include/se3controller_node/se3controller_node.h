#ifndef SE3_CONTROLLER_NODE_H
#define SE3_CONTROLLER_NODE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <se3controller/se3control.h>
#include <vehicles/multirotor/api/MultirotorRpcLibClient.hpp>
#include <tf/tf.h>






class SE3ControllerNode
{
public:
    SE3ControllerNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
    void run();

private:
 
    enum FlightState {
        TAKEOFF,       // Drone is in the process of taking off
        HOVER,         // Drone is hovering
        SE3CONTROL, // Drone is under feedback control
        LANDED        
    };
 
    void setControllerParams();
    void odometryCallback(const nav_msgs::Odometry::ConstPtr &msg);
    void targetCallback(const mavros_msgs::PositionTarget::ConstPtr &msg);
    void mainLoop(const ros::TimerEvent &event);
    void pub_attitude_cmd(const mav_control::control_cmd &cmd);
    void init_mav();

    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber odom_sub_;
    ros::Subscriber setpoint_raw_local_sub_;
    ros::Timer cmdloop_timer_;

    FlightState flight_state_; // Current flight state
    geometry_msgs::PoseStamped fcu_position_;
    geometry_msgs::TwistStamped fcu_velocity_;
    mavros_msgs::PositionTarget target_;
    mav_control::se3control controller_;
    
    mav_control::motion_target control_target_;
    mav_control::control_cmd control_cmd_;

    bool target_received_ = false;
    ros::Time last_target_time_;
    msr::airlib::MultirotorRpcLibClient client;

    double delay_time_; // Add delay time
    double Kp_x, Kp_y, Kp_z, Kv_x, Kv_y, Kv_z;
    double attctrl_tau_,norm_thrust_const_;
    double take_off_height_;
    double height_tolerance_=0.25;
    double max_fb_acc_;





};

#endif // SE3_CONTROLLER_NODE_H
