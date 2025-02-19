#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 


ros::Publisher pose_pub_;

void odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
    // 创建并发布 PoseStamped 消息
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = msg->header.stamp;
    pose_stamped.header.frame_id = msg->header.frame_id; // 保持原有的 frame_id
    pose_stamped.pose = msg->pose.pose; // 直接复制位姿信息

    // 获取原始的四元数
    tf2::Quaternion original_quat;
    tf2::fromMsg(pose_stamped.pose.orientation, original_quat);

    //=========================== IMPORTANT ===========================
    // 由于 在airsim中 enu 坐标系下，飞机方向朝前(北)时，body坐标系y轴超前（北），x轴朝东（e），z朝上（北）
    //  此时yaw为0度， 当body 顺时针旋转时,  yaw增大。
    //  由于online motion planning中所有的坐标系都是以x轴朝前的右手惯性系下建立的角度计算，
    // 所以我们这里给pose的旋转加一个90度，以让其在enu坐标系下 body 的x 朝前。
    //=========================== IMPORTANT ===========================

    // 创建一个沿 Z 轴旋转 90 度的四元数
    tf2::Quaternion rotation_quat;
    rotation_quat.setRPY(0, 0, M_PI_2); // M_PI_2 是 90 度的弧度值

    // 将原始四元数与旋转四元数相乘
    tf2::Quaternion new_quat = rotation_quat * original_quat;
    new_quat.normalize(); // 正规化四元数

    // 将新的四元数转换回 geometry_msgs::Quaternion
    pose_stamped.pose.orientation = tf2::toMsg(new_quat);

    // 发布旋转后的 PoseStamped 消息
    pose_pub_.publish(pose_stamped);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_to_pose");
    ros::NodeHandle n;


    ros::Subscriber odom_sub = n.subscribe("airsim_odom", 10, odom_callback);
    pose_pub_ = n.advertise<geometry_msgs::PoseStamped>("airsim_pose", 10);


    ros::spin();

    return 0;
}

