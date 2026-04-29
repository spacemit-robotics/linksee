#!/usr/bin/env python3

# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

"""Voice command node: receive string commands and control robot motion."""

import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import String
from geometry_msgs.msg import Twist

from tf2_ros import Buffer, TransformListener
from tf_transformations import euler_from_quaternion


class VoiceCmdNode(Node):
    """订阅字符串指令，通过TF监控位移/旋转，控制机器人运动."""

    def __init__(self):
        super().__init__('voice_cmd_node')

        # ---------- 参数 ----------
        self.declare_parameter('linear_speed', 0.2)       # m/s
        self.declare_parameter('angular_speed', 0.5)       # rad/s
        self.declare_parameter('linear_distance', 0.4)     # m
        self.declare_parameter('angular_angle', 90.0)      # degree
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('cmd_topic', 'voice_cmd')
        self.declare_parameter('control_rate', 20.0)       # Hz

        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.linear_distance = self.get_parameter('linear_distance').value
        self.angular_angle = math.radians(self.get_parameter('angular_angle').value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        cmd_topic = self.get_parameter('cmd_topic').value
        control_rate = self.get_parameter('control_rate').value

        # ---------- TF ----------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------- 发布 / 订阅 ----------
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(
            String, cmd_topic, self._voice_cmd_callback, 10
        )

        # ---------- 运动状态 ----------
        self._moving = False
        self._start_x = 0.0
        self._start_y = 0.0
        self._start_yaw = 0.0
        self._target_distance = 0.0
        self._target_angle = 0.0
        self._twist = Twist()

        # ---------- 控制定时器 ----------
        self._timer = self.create_timer(1.0 / control_rate, self._control_loop)

        self.get_logger().info(
            f'VoiceCmdNode started. Listening on "{cmd_topic}" '
            f'(commands: forward, backward, turn_left, turn_right)'
        )

    # ------------------------------------------------------------------ #
    #  TF 工具
    # ------------------------------------------------------------------ #
    def _get_current_pose(self):
        """从 TF 获取当前 (x, y, yaw)."""
        try:
            t = self.tf_buffer.lookup_transform(
                self.odom_frame, self.base_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.5)
            )
            x = t.transform.translation.x
            y = t.transform.translation.y
            q = t.transform.rotation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            return x, y, yaw
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None

    # ------------------------------------------------------------------ #
    #  指令回调
    # ------------------------------------------------------------------ #
    _CMD_MAP = {
        'forward':    ('linear',  1.0),
        'backward':   ('linear', -1.0),
        'turn_left':  ('angular', 1.0),
        'turn_right': ('angular', -1.0),
    }

    def _voice_cmd_callback(self, msg: String):
        cmd = msg.data.strip().lower()
        self.get_logger().info(f'Received command: "{cmd}"')

        if self._moving:
            self.get_logger().warn('Robot is still moving, command ignored.')
            return

        entry = self._CMD_MAP.get(cmd)
        if entry is None:
            self.get_logger().warn(
                f'Unknown command "{cmd}". '
                'Supported: forward, backward, turn_left, turn_right'
            )
            return

        pose = self._get_current_pose()
        if pose is None:
            self.get_logger().error('Cannot start motion: TF not available.')
            return

        self._start_x, self._start_y, self._start_yaw = pose
        mode, sign = entry

        twist = Twist()
        if mode == 'linear':
            twist.linear.x = sign * self.linear_speed
            self._target_distance = self.linear_distance
            self._target_angle = 0.0
        else:
            twist.angular.z = sign * self.angular_speed
            self._target_distance = 0.0
            self._target_angle = self.angular_angle

        self._twist = twist
        self._moving = True
        self.get_logger().info(
            f'Executing "{cmd}" — target: '
            f'dist={self._target_distance:.2f} m, '
            f'angle={math.degrees(self._target_angle):.1f}°'
        )

    # ------------------------------------------------------------------ #
    #  控制循环
    # ------------------------------------------------------------------ #
    def _control_loop(self):
        if not self._moving:
            return

        pose = self._get_current_pose()
        if pose is None:
            # TF 暂时不可用，保持上一次指令
            self.cmd_vel_pub.publish(self._twist)
            return

        x, y, yaw = pose

        # 判断是否到达目标
        reached = False
        if self._target_distance > 0.0:
            dx = x - self._start_x
            dy = y - self._start_y
            traveled = math.hypot(dx, dy)
            if traveled >= self._target_distance:
                reached = True
        elif self._target_angle > 0.0:
            delta_yaw = abs(self._normalize_angle(yaw - self._start_yaw))
            if delta_yaw >= self._target_angle:
                reached = True

        if reached:
            self._stop()
        else:
            self.cmd_vel_pub.publish(self._twist)

    def _stop(self):
        """停止机器人."""
        self.cmd_vel_pub.publish(Twist())
        self._moving = False
        self.get_logger().info('Motion complete — robot stopped.')

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """将角度归一化到 [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = VoiceCmdNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
