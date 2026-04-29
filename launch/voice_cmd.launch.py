#!/usr/bin/env python3

# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

"""Launch file for voice_cmd_node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('linear_speed', default_value='0.5',
                              description='Linear speed (m/s)'),
        DeclareLaunchArgument('angular_speed', default_value='1.5',
                              description='Angular speed (rad/s)'),
        DeclareLaunchArgument('linear_distance', default_value='0.4',
                              description='Distance per forward/backward (m)'),
        DeclareLaunchArgument('angular_angle', default_value='90.0',
                              description='Angle per turn (degrees)'),
        DeclareLaunchArgument('odom_frame', default_value='odom',
                              description='Odometry frame'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint',
                              description='Base frame'),
        DeclareLaunchArgument('cmd_topic', default_value='voice_cmd',
                              description='Voice command topic (std_msgs/String)'),

        Node(
            package='linksee',
            executable='voice_cmd_node.py',
            name='voice_cmd_node',
            output='screen',
            parameters=[{
                'linear_speed':    LaunchConfiguration('linear_speed'),
                'angular_speed':   LaunchConfiguration('angular_speed'),
                'linear_distance': LaunchConfiguration('linear_distance'),
                'angular_angle':   LaunchConfiguration('angular_angle'),
                'odom_frame':      LaunchConfiguration('odom_frame'),
                'base_frame':      LaunchConfiguration('base_frame'),
                'cmd_topic':       LaunchConfiguration('cmd_topic'),
            }],
        ),

    ])
