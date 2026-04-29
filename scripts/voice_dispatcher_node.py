#!/usr/bin/env python3

# Copyright 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
#
# SPDX-License-Identifier: Apache-2.0

"""Voice dispatcher: subscribe to ASR text, fuzzy match, publish commands."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import String


class VoiceDispatcherNode(Node):

    # 模糊匹配规则：关键词 -> 指令
    _MATCH_RULES = {
        'forward': ['向前走','向前进','前进', '向前', '往前', '前面', 'forward', 'go'],
        'backward': ['后退', '往后退','向后', '向后走','往后', '退', 'backward', 'back'],
        'turn_left': ['向左转','左转', '向左', '往左', '左', 'left', 'turn left'],
        'turn_right': ['向右转','右转', '向右', '往右', '右', 'right', 'turn right'],
    }

    _PATROL_RULES = {
        True: ['开始巡航', '启动巡航', '开启巡航', '开始方形巡航', '启动方形巡航', '开启方形巡航'],
        False: ['停止巡航', '结束巡航', '关闭巡航', '暂停巡航', '停止方形巡航', '结束方形巡航'],
    }

    def __init__(self):
        super().__init__('voice_dispatcher_node')

        self.declare_parameter('asr_topic', 'asr_text')
        self.declare_parameter('cmd_topic', 'voice_cmd')
        self.declare_parameter('square_waypoints_enable_topic', '/square_waypoints_enable')

        asr_topic = self.get_parameter('asr_topic').value
        cmd_topic = self.get_parameter('cmd_topic').value
        square_waypoints_enable_topic = self.get_parameter('square_waypoints_enable_topic').value

        self._pub = self.create_publisher(String, cmd_topic, 10)
        self._patrol_pub = self.create_publisher(Bool, square_waypoints_enable_topic, 10)
        self._sub = self.create_subscription(
            String, asr_topic, self._asr_callback, 10
        )

        self.get_logger().info(
            f'VoiceDispatcher started. Listening on "{asr_topic}", '
            f'publishing to "{cmd_topic}", '
            f'patrol control on "{square_waypoints_enable_topic}"'
        )

    def _asr_callback(self, msg: String):
        text = msg.data.strip()
        if not text:
            return

        self.get_logger().info(f'Received ASR: "{text}"')

        patrol_enabled = self._match_patrol_control(text)
        if patrol_enabled is not None:
            state_text = 'start' if patrol_enabled else 'stop'
            self.get_logger().info(f'Matched patrol control: "{state_text}"')
            patrol_msg = Bool()
            patrol_msg.data = patrol_enabled
            self._patrol_pub.publish(patrol_msg)

        # 模糊匹配
        cmd = self._fuzzy_match(text)
        if cmd:
            self.get_logger().info(f'Matched command: "{cmd}"')
            cmd_msg = String()
            cmd_msg.data = cmd
            self._pub.publish(cmd_msg)
        else:
            self.get_logger().warn(f'No command matched for: "{text}"')

    def _fuzzy_match(self, text: str) -> str:
        """模糊匹配：返回第一个匹配的指令，无匹配返回空字符串."""
        text_lower = text.lower()
        for cmd, keywords in self._MATCH_RULES.items():
            for keyword in keywords:
                if keyword in text_lower:
                    return cmd
        return ''

    def _match_patrol_control(self, text: str):
        """匹配巡航控制指令：返回 True/False，无匹配返回 None。"""
        text_lower = text.lower()
        for enabled, keywords in self._PATROL_RULES.items():
            for keyword in keywords:
                if keyword in text_lower:
                    return enabled
        return None


def main(args=None):
    rclpy.init(args=args)
    node = VoiceDispatcherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
