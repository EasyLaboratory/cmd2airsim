# SE3 Controller and EasySim Setup

## Overview
This guide explains how to launch the SE3 controller for multicopter takeoff, control the multicopter to follow a circular path, and receive image messages from Unreal Engine (UE) via EasySim and publish them to ROS.

## Prerequisites
- ROS environment set up and running
- SE3 controller package installed
- EasySim ROS wrapper installed
- Unreal Engine (UE) environment set up for camera feed

## Setup Instructions

### 1. Launch SE3 Controller for Takeoff
In a terminal, run the following command to launch the SE3 controller and take off the multicopter:

```bash
roslaunch se3controller se3controller.launch
```

### 2. Control the Multicopter to Follow a Circle
Open a new terminal and run the example that controls the multicopter to follow a circular path:

```bash
roslaunch se3controller flying_example.launch
```

### 3. Adjust Network Buffer Size
Increase the default network buffer size to handle larger image messages:

```bash
sudo sysctl -w net.core.rmem_default=16777216
sudo sysctl -w net.core.rmem_max=16777216
```

### 4. Launch EasySim ROS Wrapper to Receive Image Messages
In a new terminal, run the following command to start the EasySim ROS wrapper, which will receive image messages from UE and publish them to ROS:

```bash
roslaunch easysim_ros_wrapper img_ros_node.launch
```

## Troubleshooting
- Ensure the network buffer size is correctly set to avoid image transmission issues.
- Make sure the UE environment is properly configured to send the correct image feed to ROS.

---

Let me know if you need further adjustments!
