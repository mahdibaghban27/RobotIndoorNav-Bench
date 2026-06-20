#!/usr/bin/env python3
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class DynamicObstacleCommander(Node):
    def __init__(self) -> None:
        super().__init__('dynamic_obstacle_commander')
        self.publisher = self.create_publisher(Twist, '/dynamic_blocker/cmd_vel', 10)
        self.phase_period = 4.5
        self.speed = 0.45
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self._tick)
        self.get_logger().info('Dynamic obstacle commander active on /dynamic_blocker/cmd_vel')

    def _tick(self) -> None:
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        phase = math.floor(elapsed / self.phase_period) % 2
        direction = 1.0 if phase == 0 else -1.0
        msg = Twist()
        msg.linear.x = direction * self.speed
        self.publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = DynamicObstacleCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.publisher.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
