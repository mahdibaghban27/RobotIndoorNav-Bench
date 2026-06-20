#!/usr/bin/env python3
"""Dynamic obstacle commander for the house_dynamic scenario.

At t=20 s two things happen simultaneously:
  - purple_door is DELETED  → purple gap (x=[9.0,10.2] at y=4) opens.
  - blocker_1   is SPAWNED  → original door (x=[4.0,5.2] at y=4) is sealed.

The robot must detect the blocked path and replan through the purple gap.
"""
from __future__ import annotations
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity

ACTIVATE_DELAY = 20.0   # seconds

# Static SDF for the orange blocker placed at the original door gap centre.
# yaw=π/2 → long axis (1.40 m) runs along world X, covering x=[3.9, 5.3].
_BLOCKER_SDF = """\
<?xml version='1.0'?>
<sdf version='1.7'>
  <model name='blocker_1'>
    <static>true</static>
    <pose>4.600 4.000 0.450 0 0 1.5707963</pose>
    <link name='body'>
      <collision name='col'>
        <geometry><box><size>0.25 1.40 0.90</size></box></geometry>
      </collision>
      <visual name='vis'>
        <geometry><box><size>0.25 1.40 0.90</size></box></geometry>
        <material>
          <ambient>0.9 0.5 0.0 1</ambient>
          <diffuse>0.9 0.5 0.0 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


class HouseDynamicObstacleCommander(Node):
    def __init__(self) -> None:
        super().__init__('house_dynamic_obstacle_commander')
        self._spawn_cli  = self.create_client(SpawnEntity,  '/spawn_entity')
        self._delete_cli = self.create_client(DeleteEntity, '/delete_entity')
        self._start      = self.get_clock().now()
        self._triggered  = False
        self.create_timer(0.5, self._tick)
        self.get_logger().info(
            f'house_dynamic commander ready — door swap fires in {ACTIVATE_DELAY:.0f} s'
        )

    def _tick(self) -> None:
        if self._triggered:
            return
        elapsed = (self.get_clock().now() - self._start).nanoseconds * 1e-9
        if elapsed < ACTIVATE_DELAY:
            return
        self._triggered = True
        self.get_logger().info(
            f't={ACTIVATE_DELAY:.0f} s — swapping doors: blocking original, opening purple'
        )
        self._delete_purple_door()
        self._spawn_blocker()

    def _delete_purple_door(self) -> None:
        if not self._delete_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/delete_entity service unavailable')
            return
        req = DeleteEntity.Request()
        req.name = 'purple_door'
        self._delete_cli.call_async(req).add_done_callback(
            lambda f: self.get_logger().info(
                f'purple_door deleted — success={f.result().success}'
            )
        )

    def _spawn_blocker(self) -> None:
        if not self._spawn_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/spawn_entity service unavailable')
            return
        req = SpawnEntity.Request()
        req.name = 'blocker_1'
        req.xml  = _BLOCKER_SDF
        self._spawn_cli.call_async(req).add_done_callback(
            lambda f: self.get_logger().info(
                f'blocker_1 spawned — success={f.result().success}'
            )
        )


def main() -> None:
    rclpy.init()
    node = HouseDynamicObstacleCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
