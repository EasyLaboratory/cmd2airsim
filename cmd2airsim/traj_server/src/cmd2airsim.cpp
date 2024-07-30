#include <iostream>
#include <ros/ros.h>
#include <trajectory_msgs/TrajectoryPoint.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Int32.h>
#include <vehicles/multirotor/api/MultirotorRpcLibClient.hpp>
#include <math.h>
#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

// =========== Function declarations =============

#define SQ(x) ((x)*(x))

trajectory_msgs::TrajectoryPoint target_point;
msr::airlib::MultirotorRpcLibClient client;

void init_airsim_control()
{
    std::cout << "================Arm and Takeoff=================" << std::endl;

    client.enableApiControl(true);
    client.armDisarm(true);
    client.takeoffAsync(5)->waitOnLastTask();
    
    double takeoff_height=5;
    client.moveToZAsync(-takeoff_height, 1)->waitOnLastTask(); // 设定起飞高度


    std::cout << "================Takeoff Successful==================" << std::endl;
    

}


void nwu2ned(Eigen::Vector3d nwu_position, Eigen::Quaterniond nwu_quat,
             Eigen::Vector3d &ned_position, Eigen::Quaterniond &ned_quat) {
    Eigen::Matrix3d R_wu = nwu_quat.toRotationMatrix();

    Eigen::Matrix3d nwu_to_ned;
    nwu_to_ned << 1, 0, 0.0,
                  0, -1, 0.0,
                  0.0, 0.0, -1.0;

    Eigen::Matrix3d R_ed = nwu_to_ned * R_wu * nwu_to_ned.transpose();

    ned_position = nwu_to_ned * nwu_position;
    ned_quat = Eigen::Quaterniond(R_ed);
}


void traj_server_callback(const trajectory_msgs::TrajectoryPoint& msg) {

    target_point = msg;
    
    float velocity = sqrt(SQ(target_point.velocity.linear.x) +
                          SQ(target_point.velocity.linear.y) +
                          SQ(target_point.velocity.linear.z));

    Eigen::Vector3d point_position(target_point.pose.position.x,
                                   target_point.pose.position.y,
                                   target_point.pose.position.z);

    Eigen::Quaterniond point_quat(target_point.pose.orientation.w,
                                  target_point.pose.orientation.x,
                                  target_point.pose.orientation.y,
                                  target_point.pose.orientation.z);

    auto drivetrain_ = msr::airlib::DrivetrainType::MaxDegreeOfFreedom;

    // client.moveToPositionAsync(ned_target_position.x(),
    //                            ned_target_position.y(),
    //                            ned_target_position.z(),
    //                            velocity, 0.1, drivetrain_, yaw_mode_);

    client.moveToPositionAsync(point_position.x(),
                               point_position.y(),
                               point_position.z(),
                               velocity,0.1,drivetrain_, 
                               msr::airlib::YawMode(false, tf::getYaw(target_point.pose.orientation) * 180 / M_PI));
}

// =============== Main function =================
int main(int argc, char** argv) {
    ros::init(argc, argv, "cmd2airsim");
    ros::NodeHandle n;   // n 命名空间为/node_namespace launch 文件中 ns=="node_namespace"
    ros::NodeHandle nh("~");  // 命名空间为/node_namespace/node_name firefly/traj_server/

    ros::Subscriber cmd_sub_ = n.subscribe("command/trajectory", 10, &traj_server_callback);
    
    init_airsim_control();
    
    ros::spin();
}


