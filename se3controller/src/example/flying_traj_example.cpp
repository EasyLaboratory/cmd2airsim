// #include <ros/ros.h>
// #include <mavros_msgs/PositionTarget.h>
// #include <traj_msgs/Trajectory.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/Twist.h>
// #include <geometry_msgs/Accel.h>
// #include <visualization_msgs/Marker.h>
// #include <visualization_msgs/MarkerArray.h>
// #include <tf/tf.h>
// #include <cmath>

// ros::Time global_start_time;  // Global start time for trajectory generation

// double normalizeYaw(double yaw) {
//     yaw = fmod(yaw, 2 * M_PI);
//     if (yaw < 0) {
//         yaw += 2 * M_PI;
//     }
//     return yaw;
// }

// // Helper function to create TrajectoryPoint from position, velocity, and acceleration
// traj_msgs::TrajectoryPoint createTrajectoryPoint(double x, double y, double altitude, 
//                                                  double vx, double vy, double yaw, 
//                                                  double ax, double ay) {
//     traj_msgs::TrajectoryPoint point;
//     point.header.stamp = ros::Time::now(); // Current time

//     // Set position
//     point.pose.position.x = x;
//     point.pose.position.y = y;
//     point.pose.position.z = altitude;

//     // Set velocity
//     point.velocity.linear.x = vx;
//     point.velocity.linear.y = vy;
//     point.velocity.linear.z = 0; // Assume constant altitude

//     // Set acceleration
//     point.acceleration.linear.x = ax;
//     point.acceleration.linear.y = ay;
//     point.acceleration.linear.z = 0; // Assume constant altitude

//     // Set orientation (yaw)
//     tf::Quaternion quaternion;
//     quaternion.setRPY(0, 0, yaw);
//     point.pose.orientation.x = quaternion.x();
//     point.pose.orientation.y = quaternion.y();
//     point.pose.orientation.z = quaternion.z();
//     point.pose.orientation.w = quaternion.w();

//     return point;
// }

// // Helper function to create a Marker for visualization
// visualization_msgs::Marker createMarker(double x, double y, double altitude, double yaw, int id) {
//     visualization_msgs::Marker marker;
//     marker.header.frame_id = "drone_1"; // Reference frame
//     marker.header.stamp = ros::Time::now();
//     marker.ns = "trajectory_points";
//     marker.id = id;  // Unique id for each marker
//     marker.type = visualization_msgs::Marker::ARROW;
//     marker.action = visualization_msgs::Marker::ADD;
//     marker.pose.position.x = x;
//     marker.pose.position.y = y;
//     marker.pose.position.z = altitude;

//     tf::Quaternion q;
//     q.setRPY(0, 0, yaw);
//     marker.pose.orientation.x = q.x();
//     marker.pose.orientation.y = q.y();
//     marker.pose.orientation.z = q.z();
//     marker.pose.orientation.w = q.w();

//     marker.scale.x = 0.2; // Arrow length
//     marker.scale.y = 0.05; // Arrow width
//     marker.scale.z = 0.05; // Arrow height
//     marker.color.a = 1.0;  // Opacity
//     marker.color.r = 0.0;  // Red
//     marker.color.g = 0.0;  // Green
//     marker.color.b = 1.0;  // Blue

//     return marker;
// }

// void generateAndPublishEightTrajectory(ros::Publisher& traj_pub, ros::Publisher& marker_pub, 
//                                         double radius, double altitude, double omega_x, double omega_y,
//                                         double dt, double duration) {
//     // Create the trajectory message
//     traj_msgs::Trajectory traj_msg;
//     traj_msg.header.stamp = ros::Time::now();  // Set the header timestamp for the whole trajectory

//     if (global_start_time.isZero()) {
//         // If the global start time is not set, initialize it when the trajectory is first generated
//         global_start_time = ros::Time::now();
//     }

//     // Get current time and calculate time elapsed from the previous trajectory
//     ros::Time current_time = ros::Time::now();
//     double time_elapsed = (current_time - global_start_time).toSec();  // Time difference from the last trajectory

//     // Calculate number of points based on the total duration and timestep
//     int num_points = static_cast<int>(duration / dt);

//     // Clear previous points (if any) in the trajectory message
//     traj_msg.points.clear();

//     // Create a MarkerArray to hold all markers
//     visualization_msgs::MarkerArray marker_array;

//     for (int i = 0; i < num_points; ++i) {
//         // Lissajous curve equations for X and Y (8-shaped trajectory)
//         double angle = time_elapsed + i * dt;
//         double x = radius * sin(omega_x * angle);  // x position for 8-shaped path
//         double y = radius * sin(omega_y * angle);  // y position for 8-shaped path

//         // Velocity calculation (tangential to the 8-shaped curve)
//         double vx = radius * omega_x * cos(omega_x * angle);
//         double vy = radius * omega_y * cos(omega_y * angle);

//         // Centripetal acceleration (constant for motion along the curve)
//         double ax = -radius * omega_x * omega_x * sin(omega_x * angle);
//         double ay = -radius * omega_y * omega_y * sin(omega_y * angle);

//         // Calculate yaw angle
//         double yaw = atan2(vy, vx);

//         // Create the trajectory point
//         traj_msgs::TrajectoryPoint point = createTrajectoryPoint(x, y, altitude, vx, vy, yaw, ax, ay);

//         // Set the timestamp for each point based on the global start time + elapsed time
//         point.header.stamp = current_time + ros::Duration(i * dt);

//         // Add the point to the trajectory message
//         traj_msg.points.push_back(point);

//         // Create and add the Marker for this trajectory point to the MarkerArray
//         visualization_msgs::Marker marker = createMarker(x, y, altitude, yaw, i);
//         marker_array.markers.push_back(marker);
//     }

//     // Publish the trajectory
//     traj_pub.publish(traj_msg);

//     // Publish the MarkerArray
//     marker_pub.publish(marker_array);
// }

// void generateAndPublishTrajectory(ros::Publisher& traj_pub, ros::Publisher& marker_pub, 
//                                   double radius, double altitude, double omega_max, 
//                                   double acceleration, double dt, double duration) {

//     // Create the trajectory message
//     traj_msgs::Trajectory traj_msg;
//     traj_msg.header.stamp = ros::Time::now();  // Set the header timestamp for the whole trajectory

//     if (global_start_time.isZero()) {
//         // If the global start time is not set, initialize it when the trajectory is first generated
//         global_start_time = ros::Time::now();
//     }

//     // Get current time and calculate time elapsed from the previous trajectory
//     ros::Time current_time = ros::Time::now();
//     double time_elapsed = (current_time - global_start_time).toSec();  // Time difference from the last trajectory

//     // Calculate number of points based on the total duration and timestep
//     int num_points = static_cast<int>(duration / dt);

//     // Clear previous points (if any) in the trajectory message
//     traj_msg.points.clear();

//     // Create a MarkerArray to hold all markers
//     visualization_msgs::MarkerArray marker_array;

//     for (int i = 0; i < num_points; ++i) {
//         // Calculate angle based on time to ensure circular motion
//         double angle = fmod(omega_max * (time_elapsed + i * dt) + M_PI / 2, 2 * M_PI); // Ensure continuous circular motion
//         double x = radius * cos(angle);  // x position along the circle
//         double y = radius * sin(angle);  // y position along the circle

//         // Velocity calculation (tangential to the circle)
//         double vx = -radius * sin(angle) * omega_max;
//         double vy = radius * cos(angle) * omega_max;

//         // Centripetal acceleration (constant for circular motion)
//         double ax = -radius * omega_max * omega_max * cos(angle);
//         double ay = -radius * omega_max * omega_max * sin(angle);

//         // Calculate yaw angle
//         double yaw = atan2(vy, vx);

//         // Create the trajectory point
//         traj_msgs::TrajectoryPoint point = createTrajectoryPoint(x, y, altitude, vx, vy, yaw, ax, ay);

//         // Set the timestamp for each point based on the global start time + elapsed time
//         point.header.stamp = current_time + ros::Duration(i * dt);

//         // Add the point to the trajectory message
//         traj_msg.points.push_back(point);

//         // Create and add the Marker for this trajectory point to the MarkerArray
//         visualization_msgs::Marker marker = createMarker(x, y, altitude, yaw, i);
//         marker_array.markers.push_back(marker);
//     }

//     // Publish the trajectory
//     traj_pub.publish(traj_msg);

//     // Publish the MarkerArray
//     marker_pub.publish(marker_array);

// }



// void timerCallback(const ros::TimerEvent& event, ros::Publisher& traj_pub, ros::Publisher& marker_pub, 
//                    double radius, double altitude, double omega_max, double acceleration, double dt) {
//     // Generate and publish a trajectory of 3 seconds duration
//     // generateAndPublishTrajectory(traj_pub, marker_pub, radius, altitude, omega_max, acceleration, dt, 3.0);
//     generateAndPublishEightTrajectory(traj_pub, marker_pub, radius, altitude, omega_max, 0.5*omega_max, dt, 3.0);
// }


// int main(int argc, char** argv) {
//     ros::init(argc, argv, "trajectory_publisher");
//     ros::NodeHandle n;
//     ros::NodeHandle nh("~");

//     ros::Publisher traj_pub = n.advertise<traj_msgs::Trajectory>("command/trajectory", 10);
//     ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);

//     // Read parameters from ROS parameter server
//     double radius, altitude, omega_max, acceleration, dt;
//     nh.param("radius", radius, 7.0);
//     nh.param("altitude", altitude, 8.0);
//     nh.param("omega_max", omega_max, 1.2);
//     nh.param("acceleration", acceleration, 0.1);
//     nh.param("dt", dt, 0.02);

//     nh.param("omega_x", omega_max, 1.2);
//     nh.param("omega_y", omega_max, 1.5);

//     // Create a timer that triggers every second
//     ros::Timer timer = n.createTimer(ros::Duration(1.0), 
//                                      boost::bind(&timerCallback, _1, 
//                                      traj_pub, marker_pub, 
//                                      radius, altitude, omega_max,
//                                      acceleration, dt));

//     ros::spin();  // Keep the node running
//     return 0;
// }





#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
#include <traj_msgs/Trajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Accel.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <cmath>

ros::Time global_start_time;  // Global start time for trajectory generation
std::string trajectory_type="circle";

double normalizeYaw(double yaw) {
    yaw = fmod(yaw, 2 * M_PI);
    if (yaw < 0) {
        yaw += 2 * M_PI;
    }
    return yaw;
}

// Helper function to create TrajectoryPoint from position, velocity, and acceleration
traj_msgs::TrajectoryPoint createTrajectoryPoint(double x, double y, double altitude, 
                                                 double vx, double vy, double yaw, 
                                                 double ax, double ay) {
    traj_msgs::TrajectoryPoint point;
    point.header.stamp = ros::Time::now(); // Current time

    // Set position
    point.pose.position.x = x;
    point.pose.position.y = y;
    point.pose.position.z = altitude;

    // Set velocity
    point.velocity.linear.x = vx;
    point.velocity.linear.y = vy;
    point.velocity.linear.z = 0; // Assume constant altitude

    // Set acceleration
    point.acceleration.linear.x = ax;
    point.acceleration.linear.y = ay;
    point.acceleration.linear.z = 0; // Assume constant altitude

    // Set orientation (yaw)
    tf::Quaternion quaternion;
    quaternion.setRPY(0, 0, yaw);
    point.pose.orientation.x = quaternion.x();
    point.pose.orientation.y = quaternion.y();
    point.pose.orientation.z = quaternion.z();
    point.pose.orientation.w = quaternion.w();

    return point;
}

// Helper function to create a Marker for visualization
visualization_msgs::Marker createMarker(double x, double y, double altitude, double yaw, int id) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "drone_1"; // Reference frame
    marker.header.stamp = ros::Time::now();
    marker.ns = "trajectory_points";
    marker.id = id;  // Unique id for each marker
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = altitude;

    tf::Quaternion q;
    q.setRPY(0, 0, yaw);
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    marker.scale.x = 2; // Arrow length
    marker.scale.y = 1; // Arrow width
    marker.scale.z = 1; // Arrow height
    marker.color.a = 1.0;  // Opacity
    marker.color.r = 0.0;  // Red
    marker.color.g = 0.0;  // Green
    marker.color.b = 1.0;  // Blue

    return marker;
}
void generateAndPublishCircularTrajectory(ros::Publisher& traj_pub, ros::Publisher& marker_pub, 
                                           double radius, double altitude, double omega_max, 
                                           double dt, double duration) {
    traj_msgs::Trajectory traj_msg;
    traj_msg.header.stamp = ros::Time::now();

    if (global_start_time.isZero()) {
        global_start_time = ros::Time::now();
    }

    ros::Time current_time = ros::Time::now();
    double time_elapsed = (current_time - global_start_time).toSec();

    int num_points = static_cast<int>(duration / dt);
    traj_msg.points.clear();
    visualization_msgs::MarkerArray marker_array;

    for (int i = 0; i < num_points; ++i) {
        double angle = fmod(omega_max * (time_elapsed + i * dt) + M_PI / 2, 2 * M_PI);
        double x = radius * cos(angle);
        double y = radius * sin(angle);

        double vx = -radius * sin(angle) * omega_max;
        double vy = radius * cos(angle) * omega_max;

        double ax = -radius * omega_max * omega_max * cos(angle);
        double ay = -radius * omega_max * omega_max * sin(angle);

        double yaw = atan2(vy, vx);

        traj_msgs::TrajectoryPoint point = createTrajectoryPoint(x, y, altitude, vx, vy, yaw, ax, ay);
        point.header.stamp = current_time + ros::Duration(i * dt);
        traj_msg.points.push_back(point);

        visualization_msgs::Marker marker = createMarker(x, y, altitude, yaw, i);
        marker_array.markers.push_back(marker);
    }

    traj_pub.publish(traj_msg);
    marker_pub.publish(marker_array);
}

// Function to generate and publish Eight-shaped trajectory
void generateAndPublishEightTrajectory(ros::Publisher& traj_pub, ros::Publisher& marker_pub, 
                                        double radius, double altitude, double omega_x, double omega_y,
                                        double dt, double duration) {
    traj_msgs::Trajectory traj_msg;
    traj_msg.header.stamp = ros::Time::now();

    if (global_start_time.isZero()) {
        global_start_time = ros::Time::now();
    }

    ros::Time current_time = ros::Time::now();
    double time_elapsed = (current_time - global_start_time).toSec();

    int num_points = static_cast<int>(duration / dt);
    traj_msg.points.clear();
    visualization_msgs::MarkerArray marker_array;


    for (int i = 0; i < num_points; ++i) {
        double angle = time_elapsed + i * dt;
        double x = radius * sin(omega_x * angle);
        double y = radius * sin(omega_y * angle);

        double vx = radius * omega_x * cos(omega_x * angle);
        double vy = radius * omega_y * cos(omega_y * angle);

        double ax = -radius * omega_x * omega_x * sin(omega_x * angle);
        double ay = -radius * omega_y * omega_y * sin(omega_y * angle);

        double yaw = atan2(vy, vx);

        traj_msgs::TrajectoryPoint point = createTrajectoryPoint(x, y, altitude, vx, vy, yaw, ax, ay);
        point.header.stamp = current_time + ros::Duration(i * dt);
        traj_msg.points.push_back(point);

        visualization_msgs::Marker marker = createMarker(x, y, altitude, yaw, i);
        marker_array.markers.push_back(marker);
    }

    traj_pub.publish(traj_msg);
    marker_pub.publish(marker_array);
}



void timerCallback(const ros::TimerEvent& event, ros::Publisher& traj_pub, ros::Publisher& marker_pub, 
                   double radius, double altitude, double omega_x, double omega_y, double omega_max,
                   double dt) {

    if (trajectory_type == "circle") {
        
        generateAndPublishCircularTrajectory(traj_pub, marker_pub, radius, altitude, omega_max, dt, 3.0);
    
    } else if (trajectory_type == "eight") {

        generateAndPublishEightTrajectory(traj_pub, marker_pub, radius, altitude, omega_x, omega_y, dt, 3.0);
    } else {
        ROS_WARN("Invalid trajectory type specified: %s. Using default circle trajectory.", trajectory_type.c_str());
        generateAndPublishCircularTrajectory(traj_pub, marker_pub, radius, altitude, omega_max, dt, 3.0);
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_publisher");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    ros::Publisher traj_pub = n.advertise<traj_msgs::Trajectory>("command/trajectory", 10);
    ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);

    // Read parameters from ROS parameter server
    double radius, altitude, omega_max, omega_x, omega_y, acceleration, dt;
    
    nh.param("radius", radius, 7.0);
    nh.param("altitude", altitude, 8.0);
    nh.param("omega_max", omega_max, 1.2);
    nh.param("omega_x", omega_x, 1.2);
    nh.param("omega_y", omega_y, 1.5);
    nh.param("acceleration", acceleration, 0.1);
    nh.param("dt", dt, 0.05);
    nh.param("trajectory_type", trajectory_type, std::string("circle"));  // Default to circle

    ros::Timer timer = n.createTimer(ros::Duration(0.2), 
                                     boost::bind(&timerCallback, _1, 
                                     traj_pub, marker_pub, 
                                     radius, altitude, omega_x, omega_y, omega_max,
                                     dt));



    ros::spin();  // Keep the node running
    return 0;
}

