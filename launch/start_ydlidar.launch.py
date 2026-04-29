#!/usr/bin/python3

# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os


def generate_launch_description():
    share_dir = get_package_share_directory('linksee')
    parameter_file = LaunchConfiguration('params_file')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(share_dir, 'config', 'ydlidar_x3_pro.yaml'),
        description='FPath to the ROS2 parameters file to use.'
    )

    driver_node = LifecycleNode(
        package='peripherals_lidar_node',
        executable='lidar_2d_node',
        name='lidar_2d_node',
        output='screen',
        emulate_tty=True,
        parameters=[parameter_file],
        namespace='/',
    )

    return LaunchDescription([
        params_declare,
        driver_node,
    ])
