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
        self.declare_parameter('linear_tolerance', 0.05)   # m
        self.declare_parameter('angular_tolerance', 3.0)    # degree
        self.declare_parameter('linear_kp', 1.2)
        self.declare_parameter('angular_kp', 1.5)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('cmd_topic', 'voice_cmd')
        self.declare_parameter('control_rate', 20.0)       # Hz

        self.linear_speed = abs(float(self.get_parameter('linear_speed').value))
        self.angular_speed = abs(float(self.get_parameter('angular_speed').value))
        self.linear_distance = abs(float(self.get_parameter('linear_distance').value))
        self.angular_angle = math.radians(
            abs(float(self.get_parameter('angular_angle').value))
        )
        self.linear_tolerance = abs(float(self.get_parameter('linear_tolerance').value))
        self.angular_tolerance = math.radians(
            abs(float(self.get_parameter('angular_tolerance').value))
        )
        self.linear_kp = abs(float(self.get_parameter('linear_kp').value))
        self.angular_kp = abs(float(self.get_parameter('angular_kp').value))
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        cmd_topic = self.get_parameter('cmd_topic').value
        control_rate = float(self.get_parameter('control_rate').value)

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
        self._target_yaw = 0.0
        self._motion_mode = None
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
            twist.linear.x = sign * min(
                self.linear_speed, self.linear_kp * self.linear_distance
            )
            self._target_distance = sign * self.linear_distance
            self._target_angle = 0.0
            self._target_yaw = self._start_yaw
        else:
            twist.angular.z = sign * min(
                self.angular_speed, self.angular_kp * self.angular_angle
            )
            self._target_distance = 0.0
            self._target_angle = sign * self.angular_angle
            self._target_yaw = self._normalize_angle(
                self._start_yaw + self._target_angle
            )

        self._motion_mode = mode
        self._twist = twist
        self._moving = True
        self.get_logger().info(
            f'Executing "{cmd}" — target: '
            f'dist={self._target_distance:.2f} m '
            f'(tol={self.linear_tolerance:.2f} m), '
            f'angle={math.degrees(self._target_angle):.1f}° '
            f'(tol={math.degrees(self.angular_tolerance):.1f}°), '
            f'start_yaw={math.degrees(self._start_yaw):.1f}°, '
            f'target_yaw={math.degrees(self._target_yaw):.1f}°'
        )

    # ------------------------------------------------------------------ #
    #  控制循环
    # ------------------------------------------------------------------ #
    def _control_loop(self):
        if not self._moving:
            return

        pose = self._get_current_pose()
        if pose is None:
            # 精确控制依赖 TF 反馈；反馈丢失时先停车，避免无反馈超调。
            self.cmd_vel_pub.publish(Twist())
            return

        x, y, yaw = pose

        if self._motion_mode == 'linear':
            error = self._linear_error(x, y)
            if abs(error) <= self.linear_tolerance:
                self._stop(error)
                return

            twist = Twist()
            twist.linear.x = self._clamp(self.linear_kp * error, self.linear_speed)
            self._publish_motion_command(twist)
        elif self._motion_mode == 'angular':
            error = self._angular_error(yaw)
            if abs(error) <= self.angular_tolerance:
                self._stop(error)
                return

            twist = Twist()
            twist.angular.z = self._clamp(self.angular_kp * error, self.angular_speed)
            self._publish_motion_command(twist)
        else:
            self._stop()

    def _linear_error(self, x: float, y: float) -> float:
        """返回沿起始朝向的剩余有符号距离."""
        dx = x - self._start_x
        dy = y - self._start_y
        progress = dx * math.cos(self._start_yaw) + dy * math.sin(self._start_yaw)
        return self._target_distance - progress

    def _angular_error(self, yaw: float) -> float:
        """基于 odom->base_footprint 的绝对 yaw 返回剩余有符号角度."""
        return self._normalize_angle(self._target_yaw - yaw)

    def _publish_motion_command(self, twist: Twist):
        self._twist = twist
        self.cmd_vel_pub.publish(twist)

    def _stop(self, final_error=None):
        """停止机器人."""
        self.cmd_vel_pub.publish(Twist())
        self._moving = False
        self._motion_mode = None
        if final_error is None:
            self.get_logger().info('Motion complete — robot stopped.')
        elif abs(self._target_distance) > 0.0:
            self.get_logger().info(
                'Motion complete — robot stopped. '
                f'Final distance error: {abs(final_error):.3f} m'
            )
        else:
            self.get_logger().info(
                'Motion complete — robot stopped. '
                f'Final angle error: {math.degrees(abs(final_error)):.2f}°'
            )

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        """将速度限制在 [-limit, limit]."""
        return max(-limit, min(limit, value))

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
