#include "se3controller_node/se3controller_node.h"

SE3ControllerNode::SE3ControllerNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), flight_state_(TAKEOFF)
{
    init_mav();

    odom_sub_ = nh_.subscribe("odometry_topic", 1, &SE3ControllerNode::odometryCallback, this);
    setpoint_raw_local_sub_ = nh_.subscribe("position_with_yaw", 1, &SE3ControllerNode::targetCallback, this);
    cmdloop_timer_ = nh_.createTimer(ros::Duration(0.01), &SE3ControllerNode::mainLoop, this);
    
    setControllerParams();
    controller_.init_controller();
}

void SE3ControllerNode::setControllerParams()
{
    
    // 从参数服务器获取参数
    nh_private_.param("Kp_x", Kp_x, 12.0);
    nh_private_.param("Kp_y", Kp_y, 12.0);
    nh_private_.param("Kp_z", Kp_z, 10.0);
    nh_private_.param("Kv_x", Kv_x, 3.0);
    nh_private_.param("Kv_y", Kv_y, 3.0);
    nh_private_.param("Kv_z", Kv_z, 3.3);
    nh_private_.param("delay_time", delay_time_, 1.0); // Default delay time is 1 second
    nh_private_.param("attctrl_tau", attctrl_tau_, 1.0); // Default delay time is 1 second
    nh_private_.param("norm_thrust_const", norm_thrust_const_, 0.05); // Default delay time is 1 second
    nh_private_.param("max_fb_acc", max_fb_acc_, 20.0); // Default delay time is 1 second
    nh_private_.param("take_off_height", take_off_height_, 1.25); // Default delay time is 1 second



    controller_.setParameters();
    controller_.setProportionParams(Kp_x, Kp_y, Kp_z, Kv_x, Kv_y, Kv_z, attctrl_tau_,norm_thrust_const_,max_fb_acc_);
    
}

void SE3ControllerNode::odometryCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // 从odometry消息中提取位置和速度数据
    fcu_position_.pose = msg->pose.pose;
    fcu_velocity_.twist = msg->twist.twist;

    // Normalize the quaternion
    geometry_msgs::Quaternion& q = fcu_position_.pose.orientation;
    tf::Quaternion tf_q(q.x, q.y, q.z, q.w);
    tf_q.normalize();
    fcu_position_.pose.orientation.x = tf_q.x();
    fcu_position_.pose.orientation.y = tf_q.y();
    fcu_position_.pose.orientation.z = tf_q.z();
    fcu_position_.pose.orientation.w = tf_q.w();

    // std::cout<<"fcu_position_"<<fcu_position_<<std::endl;
    // std::cout<<"fcu_velocity_"<<fcu_velocity_<<std::endl;
}


void SE3ControllerNode::targetCallback(const mavros_msgs::PositionTarget::ConstPtr &msg)
{
    // std::cout <<"==============SE3ControllerNode================="<<std::endl;
    // std::cout <<"target_:  "<<target_<<std::endl;

    target_ = *msg;
    control_target_.position << target_.position.x, target_.position.y, target_.position.z;
    control_target_.velocity << target_.velocity.x, target_.velocity.y, target_.velocity.z;
    control_target_.acceleration << target_.acceleration_or_force.x, target_.acceleration_or_force.y, target_.acceleration_or_force.z;
    control_target_.yaw = target_.yaw;
    target_received_ = true;
    last_target_time_ = ros::Time::now();
}


void SE3ControllerNode::mainLoop(const ros::TimerEvent &event) {

    ros::Duration time_since_last_target = ros::Time::now() - last_target_time_;

    // Extract the current yaw angle from the drone's orientation

    // double current_yaw = tf::getYaw(fcu_position_.pose.orientation)- M_PI/2;  // - pi/2 cause enu yaw

    tf::Quaternion q(fcu_position_.pose.orientation.x,
                    fcu_position_.pose.orientation.y,
                    fcu_position_.pose.orientation.z,
                    fcu_position_.pose.orientation.w);

    // 检查四元数是否标准化
    if (std::abs(q.length() - 1.0) > 1e-3) {
        ROS_WARN("Quaternion Not Properly Normalized: length = %f", q.length());
        return; // 跳过当前帧
    }

    // 获取航向角
    double current_yaw = tf::getYaw(q) ;

    // std::cout<<"current_yaw"<<current_yaw<<std::endl;

    switch (flight_state_) {
        case TAKEOFF:
            // std::cout << "Drone is taking off." << std::endl;
            // Set the target position to the takeoff height, maintaining the current yaw angle
            control_target_.position << fcu_position_.pose.position.x, fcu_position_.pose.position.y, take_off_height_;
            control_target_.velocity.setZero();
            control_target_.acceleration.setZero();
            control_target_.yaw = current_yaw;

            // Check if the drone has reached the desired takeoff height
            if (fabs(fcu_position_.pose.position.z - take_off_height_) < height_tolerance_) {
                flight_state_ = HOVER;  // Transition to hover state when takeoff is complete
                std::cout << "=================Hovering==================" << std::endl;
            }
            break;

        case HOVER:
            // do nothing to control_target_ which means hovering.  by woods
            if (target_received_==true)
            {
                flight_state_ = SE3CONTROL;
                std::cout << "===============SE3CONTROL===============" << std::endl;
            }
            break;


        case SE3CONTROL:
            if (time_since_last_target.toSec() > delay_time_)
            {
                flight_state_ = HOVER;  
                target_received_=false;
                std::cout << "=================Hovering==================" << std::endl;
            }
            break;


        case LANDED:
            std::cout << "Drone is landed." << std::endl;
            break;

    }
    // std::cout<<"pub_attitude_cmdpub_attitude_cmd"<<std::endl;
    controller_.calculate_control(fcu_position_, fcu_velocity_, control_target_, control_cmd_);
    pub_attitude_cmd(control_cmd_);
}


void SE3ControllerNode::pub_attitude_cmd(const mav_control::control_cmd &cmd)
{
    double roll_rate = cmd.body_rate_cmd[0];
    double pitch_rate = cmd.body_rate_cmd[1];
    double yaw_rate = cmd.body_rate_cmd[2];
    double thrust = cmd.body_rate_cmd[3];

    // std::cout<<"==============thrust=========="<<thrust<<std::endl;
    // std::cout<<"thrust: "<<thrust<<std::endl;
    client.moveByAngleRatesThrottleAsync( pitch_rate, roll_rate, -yaw_rate, thrust, 0.01);
    // client.moveByAngleRatesThrottleAsync( roll_rate, -pitch_rate, -yaw_rate, thrust, 0.01);

}

void SE3ControllerNode::init_mav()
{
    std::cout << "================init_mav=================" << std::endl;
    client.confirmConnection();
    client.enableApiControl(true);
    client.armDisarm(true);
    client.takeoffAsync()->waitOnLastTask();
}

void SE3ControllerNode::run()
{
    ros::spin();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "se3_controller_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    SE3ControllerNode node(nh, nh_private);
    node.run();

    return 0;
}
