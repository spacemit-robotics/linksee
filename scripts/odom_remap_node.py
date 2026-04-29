#!/usr/bin/env python3

# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

"""Combine odom_base velocity with odom->base_footprint TF pose and publish /odom."""

from typing import Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class OdomRemapNode(Node):
    """发布融合后的 odom 消息。"""

    def __init__(self):
        super().__init__('odom_remap_node')

        self.declare_parameter('input_odom_topic', 'odom_base')
        self.declare_parameter('output_odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('tf_lookup_timeout', 0.1)

        self.input_odom_topic = self.get_parameter('input_odom_topic').value
        self.output_odom_topic = self.get_parameter('output_odom_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.tf_lookup_timeout = float(self.get_parameter('tf_lookup_timeout').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(Odometry, self.output_odom_topic, 10)
        self.subscription = self.create_subscription(
            Odometry,
            self.input_odom_topic,
            self._odom_callback,
            10,
        )

        self._tf_warned = False

        self.get_logger().info(
            f'Odom remap node started: {self.input_odom_topic} + '
            f'TF({self.odom_frame}->{self.base_frame}) -> {self.output_odom_topic}'
        )

    def _lookup_transform(self) -> Optional[TransformStamped]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=self.tf_lookup_timeout),
            )
            self._tf_warned = False
            return transform
        except TransformException as exc:
            if not self._tf_warned:
                self.get_logger().warn(
                    f'Failed to lookup TF {self.odom_frame}->{self.base_frame}: {exc}'
                )
                self._tf_warned = True
            return None

    def _odom_callback(self, msg: Odometry) -> None:
        transform = self._lookup_transform()
        if transform is None:
            return

        odom_msg = Odometry()
        odom_msg.header.stamp = msg.header.stamp
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        odom_msg.pose.pose.position.x = transform.transform.translation.x
        odom_msg.pose.pose.position.y = transform.transform.translation.y
        odom_msg.pose.pose.position.z = transform.transform.translation.z
        odom_msg.pose.pose.orientation = transform.transform.rotation
        odom_msg.pose.covariance = msg.pose.covariance

        odom_msg.twist = msg.twist

        self.publisher.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = OdomRemapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
