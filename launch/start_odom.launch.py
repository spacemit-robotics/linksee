#!/usr/bin/python3

# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share_dir = get_package_share_directory('linksee')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    configuration_directory = LaunchConfiguration(
        'configuration_directory',
        default=os.path.join(pkg_share_dir, 'config')
    )
    configuration_basename = LaunchConfiguration(
        'configuration_basename',
        default='provider_odom.lua'
    )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', configuration_basename]
        )

    odom_topic_node = Node(
        package='linksee',
        executable='odom_remap_node.py',
        name='odom_remap_node'
    )

    ld = LaunchDescription()
    ld.add_action(cartographer_node)
    ld.add_action(odom_topic_node)

    return ld
