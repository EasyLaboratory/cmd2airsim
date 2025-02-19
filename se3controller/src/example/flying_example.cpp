
#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <cmath>

double normalizeYaw(double yaw) {
    yaw = fmod(yaw, 2 * M_PI);
    if (yaw < 0) {
        yaw += 2 * M_PI;
    }
    return yaw;
}

void publishTrajectoryPoint(ros::Publisher& traj_pub, ros::Publisher& pose_pub, double x, double y, double altitude, double vx, double vy, double yaw) {
    mavros_msgs::PositionTarget point;
    point.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    point.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX | mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
                      mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    point.position.x = x;
    point.position.y = y;
    point.position.z = altitude;

    point.velocity.x = vx;
    point.velocity.y = vy;
    point.velocity.z = 0; // Assume constant altitude

    yaw = normalizeYaw(yaw);  // Normalize yaw

    point.yaw = yaw - M_PI/2.0;

    traj_pub.publish(point);

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "drone_1"; // Frame ID can be set to the appropriate frame

    pose_msg.pose.position.x = x;
    pose_msg.pose.position.y = y;
    pose_msg.pose.position.z = altitude;

    tf::Quaternion quaternion;
    quaternion.setRPY(0, 0, yaw);
    pose_msg.pose.orientation.x = quaternion.x();
    pose_msg.pose.orientation.y = quaternion.y();
    pose_msg.pose.orientation.z = quaternion.z();
    pose_msg.pose.orientation.w = quaternion.w();

    pose_pub.publish(pose_msg);
}

void flyCircle(ros::Publisher& traj_pub, ros::Publisher& pose_pub, double radius, double altitude, double omega, double dt) {
    ros::Rate rate(1.0 / dt);  // 设置发布频率
    double t = 0.0; // 初始时间

    while (ros::ok()) {
        double angle = fmod(omega * t + M_PI / 2, 2 * M_PI); // 使用 fmod 确保角度在0到2π之间
        double x = radius * cos(angle);
        double y = radius * sin(angle);

        double vx = -radius * sin(angle) * omega;
        double vy = radius * cos(angle) * omega;
        double yaw = atan2(vy, vx)+M_PI/2;

        publishTrajectoryPoint(traj_pub, pose_pub, x, y, altitude, vx, vy, yaw);

        t += dt; // 增加时间
        rate.sleep();
    }
}

void flyCircle1(ros::Publisher& traj_pub, ros::Publisher& pose_pub, double radius, double altitude, double omega, double dt) {
    ros::Rate rate(1.0 / dt);  // 设置发布频率
    double t = 0.0; // 初始时间

    while (ros::ok()) {
        double angle = omega * t; // 角度从0开始，确保从(0, 0)出发
        double x = radius * (1 - cos(angle)); // 改变轨迹使其从(0, 0发
        double y = radius * sin(angle);

        double vx = radius * omega * sin(angle);
        double vy = radius * omega * cos(angle);
        double yaw = atan2(vy, vx)+M_PI/2;

        publishTrajectoryPoint(traj_pub, pose_pub, x, y, altitude, vx, vy, yaw);

        t += dt; // 增加时间
        rate.sleep();
    }
}

void flyFigureEight(ros::Publisher& traj_pub, ros::Publisher& pose_pub, double radius, double altitude, double omega, double dt) {
    ros::Rate rate(1.0 / dt);  // 设置发布频率
    double t = 0.0; // 初始时间

    while (ros::ok()) {
        double angle = fmod(omega * t, 2 * M_PI); // 使用 fmod 确保角度在0到2π之间
        double x = radius * sin(angle);
        double y = radius * sin(2 * angle);

        double vx = radius * cos(angle) * omega;
        double vy = 2 * radius * cos(2 * angle) * omega;
        double yaw = atan2(vy, vx);
        

        publishTrajectoryPoint(traj_pub, pose_pub, x, y, altitude, vx, vy, yaw);
        t += dt; // 增加时间

        rate.sleep();
    }
}

void flyCircleWithAcceleration(ros::Publisher& traj_pub, ros::Publisher& pose_pub, double radius, double altitude, double omega_max, double acceleration, double dt) {
    ros::Rate rate(1.0 / dt);  // Set the publishing frequency
    double t = 0.0; // Initial time
    double omega = 0.0; // Start with zero angular velocity

    while (ros::ok()) {
        // Increase omega by acceleration until it reaches omega_max
        omega = std::min(omega_max, omega + acceleration * dt);

        double angle = fmod(omega * t + M_PI / 2, 2 * M_PI); // Ensure angle is between 0 and 2π
        double x = radius * cos(angle);
        double y = radius * sin(angle);

        double vx = -radius * sin(angle) * omega;
        double vy = radius * cos(angle) * omega;
        double yaw = atan2(vy, vx);

        publishTrajectoryPoint(traj_pub, pose_pub, x, y, altitude, vx, vy, yaw);

        t += dt; // Increase time
        rate.sleep();
    }
}

void flyStraightWithAcceleration(ros::Publisher& traj_pub, ros::Publisher& pose_pub, double target_speed, double acceleration, double altitude, double dt) {
    ros::Rate rate(1.0 / dt);  // Set the publishing frequency
    double t = 0.0; // Initial time
    double current_speed = 0.0; // Start with zero speed
    double x = 0.0, y = 0.0; // Start position

    while (ros::ok()) {
        // Increase current speed by acceleration until it reaches target_speed
        current_speed = std::min(target_speed, current_speed + acceleration * dt);

        y += current_speed * dt; // Update x position, assuming straight line along x-axis

        double yaw = M_PI/2; // Forward direction (along x-axis)
        publishTrajectoryPoint(traj_pub, pose_pub, x, y, altitude, 0, current_speed, yaw);

        t += dt; // Increase time
        rate.sleep();
    }
}

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "trajectory_publisher");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    ros::Publisher traj_pub = n.advertise<mavros_msgs::PositionTarget>("/command/position_target", 10);
    ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("trajectory_pose", 10);


    // 读取参数
    // double radius, altitude, omega, dt;
    // nh.param("radius", radius, 7.0);
    // nh.param("altitude", altitude, 8.0);
    // nh.param("omega", omega, 1.2);
    // nh.param("dt", dt, 0.02);

    // Uncomment one of the following lines to choose the flight pattern
    // flyCircle(traj_pub, pose_pub, radius, altitude, omega, dt);
    // flyCircle(traj_pub, pose_pub, radius, altitude, omega, dt);

    // Read parameters

    double radius, altitude, omega_max, acceleration, dt;
    nh.param("radius", radius, 7.0);
    nh.param("altitude", altitude, 8.0);
    nh.param("omega_max", omega_max, 1.2);
    nh.param("acceleration", acceleration, 0.1);
    nh.param("dt", dt, 0.02);
    
    flyCircle(traj_pub, pose_pub, radius, altitude, omega_max,  dt);

    // flyFigureEight(traj_pub, pose_pub, radius, altitude, omega_max, dt);
    // flyStraightWithAcceleration(traj_pub, pose_pub, 10.0, acceleration,altitude, dt);

    return 0;
}
