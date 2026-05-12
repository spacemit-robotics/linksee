#!/usr/bin/env python3

# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

"""
Launch file for K3 ESOS base control node.

使用 RPMsg 与 RCPU 端通信，并在 Linux 端完成里程计计算。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ================== 声明 Launch 参数 ==================
    send_hz = LaunchConfiguration('send_hz')
    odom_hz = LaunchConfiguration('odom_hz')
    cmd_vel_timeout = LaunchConfiguration('cmd_vel_timeout')
    publish_tf = LaunchConfiguration('publish_tf')
    odom_topic = LaunchConfiguration('odom_topic')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')

    wheel_radius = LaunchConfiguration('wheel_radius')
    wheel_base = LaunchConfiguration('wheel_base')
    motor1_factor = LaunchConfiguration('motor1_factor')
    motor2_factor = LaunchConfiguration('motor2_factor')
    reduction_ratio = LaunchConfiguration('reduction_ratio')
    ff_factor = LaunchConfiguration('ff_factor')
    pid_kp = LaunchConfiguration('pid_kp')
    pid_ki = LaunchConfiguration('pid_ki')
    pid_kd = LaunchConfiguration('pid_kd')
    cfg_send_on_startup = LaunchConfiguration('cfg_send_on_startup')
    feedback_enable = LaunchConfiguration('feedback_enable')

    rpmsg_ctrl_dev = LaunchConfiguration('rpmsg_ctrl_dev')
    rpmsg_data_dev = LaunchConfiguration('rpmsg_data_dev')
    rpmsg_service_name = LaunchConfiguration('rpmsg_service_name')
    rpmsg_local_addr = LaunchConfiguration('rpmsg_local_addr')
    rpmsg_remote_addr = LaunchConfiguration('rpmsg_remote_addr')

    declare_send_hz = DeclareLaunchArgument(
        'send_hz',
        default_value='20.0',
        description='Command send frequency in Hz'
    )

    declare_odom_hz = DeclareLaunchArgument(
        'odom_hz',
        default_value='50.0',
        description='Odometry publish frequency in Hz'
    )

    declare_cmd_vel_timeout = DeclareLaunchArgument(
        'cmd_vel_timeout',
        default_value='0.2',
        description='Timeout in seconds before stopping when cmd_vel is lost'
    )

    declare_publish_tf = DeclareLaunchArgument(
        'publish_tf',
        default_value='false',
        description='Whether to publish odom -> base TF'
    )

    declare_odom_topic = DeclareLaunchArgument(
        'odom_topic',
        default_value='odom_base',
        description='Odometry topic name'
    )

    declare_odom_frame = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odometry frame id'
    )

    declare_base_frame = DeclareLaunchArgument(
        'base_frame',
        default_value='base_footprint',
        description='Base frame id'
    )

    declare_wheel_radius = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.0335',
        description='Wheel radius in meters'
    )

    declare_wheel_base = DeclareLaunchArgument(
        'wheel_base',
        default_value='0.183',
        description='Wheel base (distance between wheels) in meters'
    )

    declare_motor1_factor = DeclareLaunchArgument(
        'motor1_factor',
        default_value='1.0',
        description='Speed factor for motor 1'
    )

    declare_motor2_factor = DeclareLaunchArgument(
        'motor2_factor',
        default_value='1.0',
        description='Speed factor for motor 2'
    )

    declare_reduction_ratio = DeclareLaunchArgument(
        'reduction_ratio',
        default_value='56.0',
        description='Motor reduction ratio for CFG command'
    )

    declare_ff_factor = DeclareLaunchArgument(
        'ff_factor',
        default_value='0.3',
        description='Feed-forward factor for CFG command'
    )

    declare_pid_kp = DeclareLaunchArgument(
        'pid_kp',
        default_value='0.05',
        description='PID Kp for CFG command'
    )

    declare_pid_ki = DeclareLaunchArgument(
        'pid_ki',
        default_value='0.2',
        description='PID Ki for CFG command'
    )

    declare_pid_kd = DeclareLaunchArgument(
        'pid_kd',
        default_value='0.01',
        description='PID Kd for CFG command'
    )

    declare_cfg_send_on_startup = DeclareLaunchArgument(
        'cfg_send_on_startup',
        default_value='true',
        description='Whether to send CFG command on startup'
    )

    declare_feedback_enable = DeclareLaunchArgument(
        'feedback_enable',
        default_value='true',
        description='Whether to enable motor feedback from RCPU'
    )

    declare_rpmsg_ctrl_dev = DeclareLaunchArgument(
        'rpmsg_ctrl_dev',
        default_value='/dev/rpmsg_ctrl0',
        description='RPMsg control device path'
    )

    declare_rpmsg_data_dev = DeclareLaunchArgument(
        'rpmsg_data_dev',
        default_value='/dev/rpmsg0',
        description='RPMsg data device path'
    )

    declare_rpmsg_service_name = DeclareLaunchArgument(
        'rpmsg_service_name',
        default_value='rpmsg:motor_ctrl',
        description='RPMsg endpoint service name'
    )

    declare_rpmsg_local_addr = DeclareLaunchArgument(
        'rpmsg_local_addr',
        default_value='1003',
        description='RPMsg local endpoint address on Linux side'
    )

    declare_rpmsg_remote_addr = DeclareLaunchArgument(
        'rpmsg_remote_addr',
        default_value='1002',
        description='RPMsg remote endpoint address on RCPU side'
    )

    # ================== TF 静态变换 ==================
    tf2_node_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_base',
        arguments=['0.0', '0.0', '0.0001', '0.0', '0.0', '0.0',
                   'base_footprint', 'base_link'],
    )

    tf2_node_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_pub_base_to_laser',
        arguments=['0.02', '0.0', '0.20', '3.14159', '0.0', '0.0',
                   'base_link', 'laser_link'],
    )

    tf2_node_laser_rplidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_pub_base_to_laser_rplidar',
        arguments=['0.0', '0.0', '0.20', '3.14159', '0.0', '0.0',
                   'base_link', 'rplidar_link'],
    )

    tf2_node_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_pub_base_to_imu',
        arguments=['0.0', '0.0', '0.01', '0.0', '0.0', '0.0',
                   'base_link', 'imu_link'],
    )

    tf2_node_rgbd = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_rgbd',
        arguments=['0.06', '0.0', '0.06', '0.0', '0.0', '0.0',
                   'base_link', 'camera_link'],
    )

    # ================== K3 ESOS 底盘控制节点 ==================
    rpmsg_legacy_node = Node(
        package='base',
        executable='esos_base_control_node',
        name='esos_base_control_node',
        output='screen',
        parameters=[{
            'send_hz': send_hz,
            'odom_hz': odom_hz,
            'cmd_vel_timeout': cmd_vel_timeout,
            'publish_tf': publish_tf,
            'odom_topic': odom_topic,
            'odom_frame': odom_frame,
            'base_frame': base_frame,
            'wheel_radius': wheel_radius,
            'wheel_base': wheel_base,
            'motor1_factor': motor1_factor,
            'motor2_factor': motor2_factor,
            'reduction_ratio': reduction_ratio,
            'ff_factor': ff_factor,
            'pid_kp': pid_kp,
            'pid_ki': pid_ki,
            'pid_kd': pid_kd,
            'cfg_send_on_startup': cfg_send_on_startup,
            'feedback_enable': feedback_enable,
            'rpmsg_ctrl_dev': rpmsg_ctrl_dev,
            'rpmsg_data_dev': rpmsg_data_dev,
            'rpmsg_service_name': rpmsg_service_name,
            'rpmsg_local_addr': rpmsg_local_addr,
            'rpmsg_remote_addr': rpmsg_remote_addr,
        }]
    )

    return LaunchDescription([
        declare_send_hz,
        declare_odom_hz,
        declare_cmd_vel_timeout,
        declare_publish_tf,
        declare_odom_topic,
        declare_odom_frame,
        declare_base_frame,
        declare_wheel_radius,
        declare_wheel_base,
        declare_motor1_factor,
        declare_motor2_factor,
        declare_reduction_ratio,
        declare_ff_factor,
        declare_pid_kp,
        declare_pid_ki,
        declare_pid_kd,
        declare_cfg_send_on_startup,
        declare_feedback_enable,
        declare_rpmsg_ctrl_dev,
        declare_rpmsg_data_dev,
        declare_rpmsg_service_name,
        declare_rpmsg_local_addr,
        declare_rpmsg_remote_addr,

        tf2_node_base,
        tf2_node_laser,
        tf2_node_laser_rplidar,
        tf2_node_imu,
        tf2_node_rgbd,
        rpmsg_legacy_node,
    ])
