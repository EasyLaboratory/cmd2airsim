#include <ros/ros.h>
#include <trajectory_msgs/Trajectory.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Accel.h>

// 全局变量
ros::Publisher cmd_pub_;
trajectory_msgs::Trajectory temp_traj;
trajectory_msgs::TrajectoryPoint target_point;
bool traj_ready = false;
int traj_count = 0;
int traj_size = 0;


void traj_server_callback(const trajectory_msgs::Trajectory& msg) 
{
    
    temp_traj = msg;
    traj_size = temp_traj.points.size();
    traj_ready = true;

    // 获取当前时间
    ros::Time current_time = ros::Time::now();

    // 确定从哪个点开始
    traj_count = 0;
    while (traj_count < traj_size && 
           (temp_traj.points[traj_count].header.stamp) < current_time) 
    {
        traj_count++;
    }
    
    // 如果所有点都在过去，则从最后一个点开始
    if (traj_count >= traj_size) 
    {
        traj_ready=false;
        ROS_INFO_STREAM("\033[31m Current Time > The Last Trajectory Point Time\033[0m");
        return;
    }

    target_point.header = temp_traj.header;
}


void cmd_pub_callback(const ros::TimerEvent& e)
{
    if (traj_ready)
    {

        // 获取当前时间
        ros::Time current_time = ros::Time::now();
        // 获取当前轨迹点的时间戳
        ros::Time point_time = temp_traj.points[traj_count].header.stamp;

        // 检查当前时间是否超过轨迹点的时间戳
        if (current_time >= point_time)
        {
            target_point = temp_traj.points[traj_count];
            cmd_pub_.publish(target_point);
            traj_count++;
        }

    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "traj_server");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    std::cout << "===============traj_server created=============" << std::endl;

    cmd_pub_ = n.advertise<trajectory_msgs::TrajectoryPoint>("command/trajectory", 10);

    ros::Subscriber traj_sub_ = nh.subscribe("trajectory", 10, traj_server_callback);
    ros::Timer cmd_pub_timer = nh.createTimer(ros::Duration(0.05), cmd_pub_callback);

    ros::spin();
}
