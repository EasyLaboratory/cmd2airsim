#include <ros/ros.h>
#include <traj_msgs/Trajectory.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Accel.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <visualization_msgs/Marker.h>

 
ros::Publisher cmd_pub_;
ros::Publisher marker_pub_;
traj_msgs::Trajectory temp_traj;
traj_msgs::TrajectoryPoint target_point;
bool traj_ready = false;
int traj_count = 0;
int traj_size = 0;


// Trajectory message callback
void traj_server_callback(const traj_msgs::Trajectory& msg) 
{
    temp_traj = msg;
    traj_size = temp_traj.points.size();
    if (traj_size>0)
    {
        traj_ready = true;
        ROS_INFO_STREAM("\033[31m traj is ready \033[0m");

    }

    ros::Time current_time = ros::Time::now();
    traj_count = 0;

    // Find the point in the trajectory that corresponds to the current time
    while (traj_count < traj_size && temp_traj.points[traj_count].header.stamp < current_time) 
    {
        traj_count++;
    }
    
    if (traj_count >= traj_size) 
    {
        traj_ready = false;
        ROS_INFO_STREAM("\033[31m Current Time > The Last Trajectory Point Time\033[0m");
        return;
    }

    target_point.header = temp_traj.header;
}


// Adjust yaw by subtracting 90 degrees (π/2 radians)
double adjust_yaw(double yaw)
{
    return yaw - M_PI_2;
}

// Create PositionTarget message
mavros_msgs::PositionTarget create_position_target(const traj_msgs::TrajectoryPoint& point)
{
    mavros_msgs::PositionTarget position_target;
    position_target.header = point.header;
    position_target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    position_target.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                                mavros_msgs::PositionTarget::IGNORE_VY |
                                mavros_msgs::PositionTarget::IGNORE_VZ |
                                mavros_msgs::PositionTarget::IGNORE_AFX |
                                mavros_msgs::PositionTarget::IGNORE_AFY |
                                mavros_msgs::PositionTarget::IGNORE_AFZ |
                                mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    // Set position, velocity, and acceleration
    position_target.position = point.pose.position;
    position_target.velocity = point.velocity.linear;
    position_target.acceleration_or_force = point.acceleration.linear;

    // Adjust yaw using the helper function
    tf2::Quaternion quat(point.pose.orientation.x, 
                         point.pose.orientation.y, 
                         point.pose.orientation.z, 
                         point.pose.orientation.w);
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    position_target.yaw = adjust_yaw(yaw);

    return position_target;
}


// Create marker for visualization
visualization_msgs::Marker create_marker(const traj_msgs::TrajectoryPoint& point)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "drone_1"; // or the appropriate frame
    marker.header.stamp = ros::Time::now();
    marker.ns = "position_target";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = point.pose;
    marker.scale.x = 2;  // Arrow length
    marker.scale.y = 0.9;  // Arrow width
    marker.scale.z = 0.9;  // Arrow height
    marker.color.a = 1.0;  // Opacity
    marker.color.r = 0.0;  // Red
    marker.color.g = 1.0;  // Green
    marker.color.b = 0.0;  // Blue

    return marker;
}

// Command publish callback
void cmd_pub_callback(const ros::TimerEvent& e)
{
    if (traj_ready)
    {
        ros::Time current_time = ros::Time::now();
        ros::Time point_time = temp_traj.points[traj_count].header.stamp;


        if (current_time >= point_time)
        {
            target_point = temp_traj.points[traj_count];

            // Create and publish PositionTarget message
            mavros_msgs::PositionTarget position_target = create_position_target(target_point);
            cmd_pub_.publish(position_target);

            // Create and publish visualization marker
            visualization_msgs::Marker marker = create_marker(target_point);
            marker_pub_.publish(marker);

            // Increment the trajectory count
            traj_count++;
        }

        // If all trajectory points have been processed
        if (traj_count >= traj_size)
        {
            traj_ready = false;
            ROS_INFO_STREAM("\033[33mAll trajectory points have been published.\033[0m");
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "traj_server");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    std::cout << "=============== traj_server created =============" << std::endl;
    // Initialize publishers
    cmd_pub_ = n.advertise<mavros_msgs::PositionTarget>("cmd2controller", 10);
    marker_pub_ = n.advertise<visualization_msgs::Marker>("curr_target", 10);

    // Initialize subscriber for trajectory
    ros::Subscriber traj_sub_ = n.subscribe("input_trajectory_topic", 10, traj_server_callback);

    // Initialize timer to periodically call the cmd_pub_callback
    ros::Timer cmd_pub_timer = nh.createTimer(ros::Duration(0.005), cmd_pub_callback);

    ros::spin();
}
