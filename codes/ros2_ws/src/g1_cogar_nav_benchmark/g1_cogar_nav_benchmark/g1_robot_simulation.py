#!/usr/bin/env python3
import math
from typing import List

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class G1RobotSimulation(Node):
    def __init__(self) -> None:
        super().__init__('g1_robot_simulation')

        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('traj_topic', '/set_joint_trajectory')
        self.declare_parameter('joint_state_topic', '/joint_states')

        self.declare_parameter('timer_period', 0.10)

        self.declare_parameter('base_freq', 0.1)
        self.declare_parameter('speed_freq_gain', 0.70)
        self.declare_parameter('turn_freq_gain', 0.15)

        self.declare_parameter('hip_amp_min', 0.10)
        self.declare_parameter('hip_amp_max', 0.22)

        self.declare_parameter('knee_amp_min', 0.16)
        self.declare_parameter('knee_amp_max', 0.36)

        self.declare_parameter('ankle_amp_min', 0.06)
        self.declare_parameter('ankle_amp_max', 0.12)

        self.declare_parameter('arm_amp_min', 0.06)
        self.declare_parameter('arm_amp_max', 0.14)

        self.declare_parameter('yaw_bias_gain', 0.015)

        self.declare_parameter('max_linear_speed', 0.18)
        self.declare_parameter('max_angular_speed', 0.50)

        self.declare_parameter('move_linear_threshold', 0.015)
        self.declare_parameter('move_angular_threshold', 0.04)

        self.declare_parameter('filter_alpha', 0.20)
        self.declare_parameter('stand_knee', 0.10)
        self.declare_parameter('stand_ankle', -0.05)

        self.phase = 0.0
        self.last_cmd = Twist()
        self.filtered_v = 0.0
        self.filtered_w = 0.0

        self.timer_period = float(self.get_parameter('timer_period').value)
        self.filter_alpha = float(self.get_parameter('filter_alpha').value)

        self.base_freq = float(self.get_parameter('base_freq').value)
        self.speed_freq_gain = float(self.get_parameter('speed_freq_gain').value)
        self.turn_freq_gain = float(self.get_parameter('turn_freq_gain').value)

        self.hip_amp_min = float(self.get_parameter('hip_amp_min').value)
        self.hip_amp_max = float(self.get_parameter('hip_amp_max').value)

        self.knee_amp_min = float(self.get_parameter('knee_amp_min').value)
        self.knee_amp_max = float(self.get_parameter('knee_amp_max').value)

        self.ankle_amp_min = float(self.get_parameter('ankle_amp_min').value)
        self.ankle_amp_max = float(self.get_parameter('ankle_amp_max').value)

        self.arm_amp_min = float(self.get_parameter('arm_amp_min').value)
        self.arm_amp_max = float(self.get_parameter('arm_amp_max').value)

        self.yaw_bias_gain = float(self.get_parameter('yaw_bias_gain').value)

        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)

        self.move_linear_threshold = float(self.get_parameter('move_linear_threshold').value)
        self.move_angular_threshold = float(self.get_parameter('move_angular_threshold').value)

        self.cmd_topic = str(self.get_parameter('cmd_topic').value)
        self.traj_topic = str(self.get_parameter('traj_topic').value)
        self.joint_state_topic = str(self.get_parameter('joint_state_topic').value)

        self.cmd_sub = self.create_subscription(
            Twist,
            self.cmd_topic,
            self.cmd_callback,
            10
        )

        self.traj_pub = self.create_publisher(
            JointTrajectory,
            self.traj_topic,
            10
        )

        self.joint_state_pub = self.create_publisher(
            JointState,
            self.joint_state_topic,
            10
        )

        self.timer = self.create_timer(self.timer_period, self.on_timer)

        self.joint_names: List[str] = [
            'left_hip_pitch_joint',
            'right_hip_pitch_joint',
            'left_knee_joint',
            'right_knee_joint',
            'left_ankle_pitch_joint',
            'right_ankle_pitch_joint',
            'left_shoulder_pitch_joint',
            'right_shoulder_pitch_joint',
        ]

        stand_knee = float(self.get_parameter('stand_knee').value)
        stand_ankle = float(self.get_parameter('stand_ankle').value)

        self.current_positions = [0.0] * len(self.joint_names)
        self.stand_positions = [
            0.0,
            0.0,
            stand_knee,
            stand_knee,
            stand_ankle,
            stand_ankle,
            0.0,
            0.0,
        ]

    def cmd_callback(self, msg: Twist) -> None:
        self.last_cmd = msg

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    @staticmethod
    def lerp(a: float, b: float, t: float) -> float:
        return a + (b - a) * t

    def publish_joint_states(self, positions: List[float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.joint_names)
        msg.position = list(positions)
        self.joint_state_pub.publish(msg)

    def publish_trajectory(self, positions: List[float]) -> None:
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.joint_names = list(self.joint_names)

        pt = JointTrajectoryPoint()
        pt.positions = list(positions)
        pt.time_from_start.sec = 0
        pt.time_from_start.nanosec = int(self.timer_period * 1e9)

        msg.points = [pt]
        self.traj_pub.publish(msg)

    def on_timer(self) -> None:
        raw_v = float(self.last_cmd.linear.x)
        raw_w = float(self.last_cmd.angular.z)

        self.filtered_v = self.lerp(self.filtered_v, raw_v, self.filter_alpha)
        self.filtered_w = self.lerp(self.filtered_w, raw_w, self.filter_alpha)

        v = self.filtered_v
        w = self.filtered_w

        av = abs(v)
        aw = abs(w)

        moving = av > self.move_linear_threshold or aw > self.move_angular_threshold

        if moving:
            stride_ratio = self.clamp(v / self.max_linear_speed, -1.0, 1.0)
            turn_ratio = self.clamp(w / self.max_angular_speed, -1.0, 1.0)

            speed_scale = self.clamp(av / self.max_linear_speed, 0.15, 1.0)
            turn_scale = self.clamp(aw / self.max_angular_speed, 0.0, 1.0)

            freq = (
                self.base_freq
                + self.speed_freq_gain * self.clamp(av / self.max_linear_speed, 0.0, 1.0)
                + self.turn_freq_gain * turn_scale
            )

            self.phase = (
                self.phase + self.timer_period * freq * 2.0 * math.pi
            ) % (2.0 * math.pi)

            s = math.sin(self.phase)
            s_opp = math.sin(self.phase + math.pi)

            hip_amp = self.lerp(self.hip_amp_min, self.hip_amp_max, speed_scale)
            knee_amp = self.lerp(self.knee_amp_min, self.knee_amp_max, speed_scale)
            ankle_amp = self.lerp(self.ankle_amp_min, self.ankle_amp_max, speed_scale)
            arm_amp = self.lerp(self.arm_amp_min, self.arm_amp_max, speed_scale)

            yaw_bias = self.yaw_bias_gain * turn_ratio

            lhip = hip_amp * stride_ratio * s + yaw_bias
            rhip = hip_amp * stride_ratio * s_opp - yaw_bias

            lknee = 0.08 + max(0.0, knee_amp * (0.5 + 0.5 * s_opp))
            rknee = 0.08 + max(0.0, knee_amp * (0.5 + 0.5 * s))

            lankle = -0.04 - ankle_amp * stride_ratio * s - 0.16 * lknee
            rankle = -0.04 - ankle_amp * stride_ratio * s_opp - 0.16 * rknee

            larm = arm_amp * s_opp
            rarm = arm_amp * s

            positions = [
                lhip,
                rhip,
                lknee,
                rknee,
                lankle,
                rankle,
                larm,
                rarm,
            ]
        else:
            self.phase *= 0.90
            positions = [
                self.lerp(cur, target, 0.20)
                for cur, target in zip(self.current_positions, self.stand_positions)
            ]

            if all(abs(a - b) < 1e-3 for a, b in zip(positions, self.stand_positions)):
                positions = list(self.stand_positions)

        self.current_positions = positions
        self.publish_joint_states(positions)
        self.publish_trajectory(positions)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = G1RobotSimulation()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()