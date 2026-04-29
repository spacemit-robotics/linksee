# linksee

## Overview
`linksee` is a ROS 2-based mobile robot foundational package. It provides chassis serial control, odometry publishing, static transform support, and launch capabilities for 2D LiDAR and mapping workflows. Its main purpose is to help developers quickly establish the basic control-to-perception loop, from chassis driver integration to SLAM-related bring-up, with minimal setup cost.

## Features
- Supports quick startup of chassis control, 2D LiDAR, and Cartographer mapping related nodes through Launch files.
- Supports tuning of serial port, wheel diameter, wheel base, PID, feedforward, and other control parameters through Launch arguments.
- Supports launching LiDAR odometry for the robot platform.

## Quick Start
The following steps help developers complete build and basic runtime verification of the `linksee` package in the shortest path.

### Environment Preparation
- ROS 2 runtime environment is installed on K3, and basic environment variables have been initialized.
- Python runtime dependencies are available, including `rclpy`, `serial`, and related ROS 2 Python runtime packages.
- The chassis is connected, and the device node is available if serial communication is used, for example `/dev/ttyACM0`.
- If LiDAR or mapping features are needed, prepare the corresponding drivers and `cartographer_ros` related dependencies additionally.

### Build
Build the ROS 2 workspace from the workspace root and make sure the `linksee` package is compiled and installed correctly.

### Run Examples
- Start chassis control and odometry nodes: use `base_control_esos.launch.py`.
- Start 2D LiDAR: use `start_rplidar.launch.py` or `start_ydlidar.launch.py`.
- Start LiDAR odometry: use `start_odom.launch.py`.

It is recommended to verify in the order of "chassis control node → sensor node → mapping node", and focus on checking whether `/cmd_vel`, `/odom`, the TF tree, and laser scan topics are working properly.

## Detailed Usage
See the official documentation.

## FAQ
- Chassis does not move: check whether `cmd_vel` is being published correctly and whether the chassis controller serial protocol matches the current node implementation.
- Abnormal `odom` data: verify `wheel_diameter`, `wheel_base`, encoder parameters, gear ratio, and motor direction configuration carefully.
- TF display is abnormal: confirm that `publish_tf`, `odom_frame`, and `base_frame` are consistent with the rest of the system.
- Poor mapping quality: check the LiDAR mounting pose, static TF, odometry accuracy, and whether the Cartographer configuration matches the actual hardware.

## Contributing
Contributions are welcome through issues, documentation updates, parameter optimization, feature enhancement, and bug fixes.

Before contributing, it is recommended to:
- clarify the modification goal and applicable hardware scope;
- provide necessary configuration notes and validation results;
- avoid changes that are incompatible with the existing Launch and parameter system.

## License
Source file headers in this component declare Apache-2.0. The final license terms are subject to the `LICENSE` file in this directory.
