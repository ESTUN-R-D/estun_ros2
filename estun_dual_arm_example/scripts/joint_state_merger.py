#!/usr/bin/env python3

from typing import Dict, Optional, Sequence

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def _is_complete_or_empty(
    values: Sequence[float], expected_length: int
) -> bool:
    return len(values) in (0, expected_length)


def is_valid_joint_state(msg: JointState) -> bool:
    expected_length = len(msg.name)
    if expected_length == 0 or len(msg.position) != expected_length:
        return False
    if not _is_complete_or_empty(msg.velocity, expected_length):
        return False
    if not _is_complete_or_empty(msg.effort, expected_length):
        return False
    return True


def merge_joint_states(messages: Sequence[JointState]) -> Optional[JointState]:
    if not messages:
        return None

    merged = JointState()
    include_velocity = all(
        len(msg.velocity) == len(msg.name) for msg in messages
    )
    include_effort = all(len(msg.effort) == len(msg.name) for msg in messages)

    for msg in messages:
        merged.name.extend(msg.name)
        merged.position.extend(msg.position)
        if include_velocity:
            merged.velocity.extend(msg.velocity)
        if include_effort:
            merged.effort.extend(msg.effort)

    return merged


class JointStateMerger(Node):
    def __init__(self) -> None:
        super().__init__("joint_state_merger")
        self._required_arm_names = ("rarm", "larm")
        self._joint_states: Dict[str, JointState] = {}
        self._invalid_joint_state_logged: Dict[str, bool] = {}
        self.declare_parameter("rarm_joint_states_topic", "/rarm/joint_states")
        self.declare_parameter("larm_joint_states_topic", "/larm/joint_states")
        rarm_joint_states_topic = (
            self.get_parameter("rarm_joint_states_topic")
            .get_parameter_value()
            .string_value
        )
        larm_joint_states_topic = (
            self.get_parameter("larm_joint_states_topic")
            .get_parameter_value()
            .string_value
        )

        self.create_subscription(
            JointState, rarm_joint_states_topic, self._store_rarm, 10
        )
        self.create_subscription(
            JointState, larm_joint_states_topic, self._store_larm, 10
        )
        self._publisher = self.create_publisher(
            JointState, "/joint_states", 10
        )
        self.create_timer(0.05, self._publish_merged)

    def _store_rarm(self, msg: JointState) -> None:
        self._store_joint_state("rarm", msg)

    def _store_larm(self, msg: JointState) -> None:
        self._store_joint_state("larm", msg)

    def _store_joint_state(self, arm_name: str, msg: JointState) -> None:
        if is_valid_joint_state(msg):
            self._joint_states[arm_name] = msg
            self._invalid_joint_state_logged.pop(arm_name, None)
            return

        # /joint_states 是 MoveIt 的公共输入。这里收到单臂坏消息时，
        # 保留上一帧有效缓存，避免全局消息缺半套关节后让可视化/状态监控
        # 把缺失关节回退到默认零位。
        if self._invalid_joint_state_logged.get(arm_name, False):
            return
        self.get_logger().warning(
            (
                f"{arm_name} joint_states 消息长度非法，已忽略："
                "需要满足 name/position 等长，"
                "velocity/effort 为空或与 name 等长。"
            )
        )
        self._invalid_joint_state_logged[arm_name] = True

    def _publish_merged(self) -> None:
        # MoveIt 的机器人状态监控订阅全局 /joint_states，
        # 因此这里按固定左右臂顺序合并，并要求双臂缓存都已经准备好，
        # 避免启动初期只发布半套关节状态。
        if not all(
            arm_name in self._joint_states
            for arm_name in self._required_arm_names
        ):
            return
        messages = [
            self._joint_states[arm_name]
            for arm_name in self._required_arm_names
        ]
        merged = merge_joint_states(messages)
        if merged is None:
            return

        merged.header.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(merged)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = JointStateMerger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
