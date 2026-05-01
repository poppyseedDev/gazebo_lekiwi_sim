#!/usr/bin/env python3
"""Drive the LeKiwi through small_house.sdf to build a SLAM map.

Pre-req: sim + slam_toolbox already running.
  ros2 launch lekiwi_gazebo sim_bringup.launch.py world:=small_house.sdf
  ros2 launch lekiwi_gazebo slam.launch.py

Then:
  ros2 run lekiwi_gazebo auto_map.py

The script drives a holonomic (translation-only) tour through the 3 rooms,
returns to the start for loop closure, then saves the map to
~/lekiwi_ws/src/lekiwi_ros2/lekiwi_navigation/maps/small_house.{pgm,yaml}.
"""

import math
import os
import subprocess
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


# Waypoints in world (odom) frame. Each was checked against small_house.sdf
# obstacles (sofa, coffee table, bookshelf, bed, counter, stool) to ensure
# the target itself is in free space. Furniture footprints in small_house:
#   sofa         (-3.4, -1.8)  size 1.0 x 0.6   → x∈[-3.9,-2.9] y∈[-2.1,-1.5]
#   coffee_table (-2.5, -0.5)  size 0.8 x 0.5   → x∈[-2.9,-2.1] y∈[-0.75,-0.25]
#   bookshelf    (-3.7,  2.4)  size 0.4 x 1.0   → x∈[-3.9,-3.5] y∈[ 1.9, 2.9]
#   bed          ( 2.5,  2.0)  size 1.4 x 1.9   → x∈[ 1.8, 3.2] y∈[ 1.05,2.95]
#   counter      ( 2.0, -2.6)  size 2.5 x 0.7   → x∈[ 0.75,3.25] y∈[-2.95,-2.25]
#   stool        ( 1.5, -1.7)  cyl r 0.18       → ~x∈[1.32,1.68] y∈[-1.88,-1.52]
WAYPOINTS = [
    # Living room sweep (avoid sofa west, coffee table, bookshelf NW)
    (-1.5,  0.0),
    (-2.5,  2.5),    # NW (north of bookshelf row)
    (-1.0,  2.5),    # near north wall
    (-1.0, -2.5),    # diagonal across to south
    (-2.5, -2.7),    # SW (south of sofa)
    (-1.5,  0.0),    # back toward hallway
    # Through hallway
    ( 0.0,  0.0),
    # Bedroom sweep (avoid bed which fills NE corner)
    ( 0.5,  1.0),    # bedroom W
    ( 0.5,  2.5),    # bedroom NW corner
    ( 1.5,  2.5),    # north of bed
    ( 3.5,  2.5),    # NE behind bed
    ( 3.5,  0.5),    # near bedroom→kitchen doorway
    # Through doorway to kitchen (avoid counter S, stool middle)
    ( 3.5, -0.5),
    ( 3.5, -1.5),    # kitchen E
    ( 0.5, -1.0),    # kitchen NW (north of counter)
    ( 0.5, -1.8),    # kitchen W (north of counter, west of stool)
    # Loop-closure: back through hallway
    ( 0.0,  0.0),
    (-1.5,  0.0),
    ( 0.0,  0.0),
]

MAP_OUT = os.path.expanduser(
    '~/lekiwi_ws/src/lekiwi_ros2/lekiwi_navigation/maps/small_house'
)


class AutoMapper(Node):
    def __init__(self):
        super().__init__('auto_mapper')
        # use_sim_time so timeouts/loops respect /clock from gz
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.parameter.Parameter.Type.BOOL, True
        )])
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.x = self.y = self.yaw = 0.0
        self.has_odom = False

    def _odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # yaw from quaternion (planar)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        self.has_odom = True

    def stop(self):
        self.cmd_pub.publish(Twist())

    def wait_for_odom(self, timeout=10.0):
        start = time.monotonic()
        while rclpy.ok() and not self.has_odom:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - start > timeout:
                raise RuntimeError('no /odom received — is the sim running?')

    def drive_to(self, tx, ty, max_lin=0.25, tol=0.15, timeout=25.0,
                 stuck_window=4.0, stuck_dist=0.05):
        """Holonomic translation toward (tx, ty) in world frame.

        Velocity is rotated from world frame into base frame using current yaw,
        so the robot strafes/drives without rotating.

        Quits early if the robot makes less than `stuck_dist` of progress
        (toward the goal) in any `stuck_window` seconds — catches "robot
        wedged against a wall" without burning the full timeout.
        """
        self.get_logger().info(f'→ waypoint ({tx:+.1f}, {ty:+.1f})')
        start = self.get_clock().now()
        last_check_t = start
        last_dist = math.hypot(tx - self.x, ty - self.y)
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            dx = tx - self.x
            dy = ty - self.y
            dist = math.hypot(dx, dy)
            if dist < tol:
                break
            # Slow down near the goal
            speed = min(max_lin, 0.6 * dist + 0.05)
            ux = dx / dist
            uy = dy / dist
            # World → base: rotate by -yaw
            c = math.cos(-self.yaw)
            s = math.sin(-self.yaw)
            cmd = Twist()
            cmd.linear.x = speed * (ux * c - uy * s)
            cmd.linear.y = speed * (ux * s + uy * c)
            self.cmd_pub.publish(cmd)
            now = self.get_clock().now()
            elapsed = (now - start).nanoseconds / 1e9
            since_check = (now - last_check_t).nanoseconds / 1e9
            if since_check >= stuck_window:
                progress = last_dist - dist
                if progress < stuck_dist:
                    self.get_logger().warn(
                        f'stuck at ({self.x:+.2f}, {self.y:+.2f}) — '
                        f'{progress:+.2f}m progress in {since_check:.1f}s; '
                        f'skipping'
                    )
                    break
                last_check_t = now
                last_dist = dist
            if elapsed > timeout:
                self.get_logger().warn(
                    f'timeout at ({self.x:+.2f}, {self.y:+.2f}); skipping'
                )
                break
        self.stop()
        # Brief pause so SLAM ingests scans from this pose
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)


def save_map(node):
    """Save the current /map. slam_toolbox publishes /map every
    map_update_interval (5s in our config), so the saver needs a long-enough
    timeout AND --ros-args because rclpy CLI args can't merge by default."""
    node.get_logger().info(f'saving map to {MAP_OUT}.{{pgm,yaml}}')
    os.makedirs(os.path.dirname(MAP_OUT), exist_ok=True)
    res = subprocess.run(
        ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
         '-f', MAP_OUT,
         '--ros-args',
         '-p', 'use_sim_time:=true',
         '-p', 'save_map_timeout:=15.0'],
        capture_output=True, text=True, timeout=30,
    )
    if res.returncode != 0 or 'Failed to spin' in res.stdout:
        node.get_logger().error(f'map_saver failed:\n{res.stdout}\n{res.stderr}')
    else:
        node.get_logger().info('map saved.')


def main():
    rclpy.init()
    node = AutoMapper()
    try:
        node.get_logger().info('waiting for /odom...')
        node.wait_for_odom()
        node.get_logger().info('letting SLAM warm up for 3s...')
        for _ in range(30):
            rclpy.spin_once(node, timeout_sec=0.1)
        for wp in WAYPOINTS:
            node.drive_to(*wp)
        node.stop()
        # Final hold so SLAM finishes loop closure
        node.get_logger().info('final settle for 5s...')
        for _ in range(50):
            rclpy.spin_once(node, timeout_sec=0.1)
        save_map(node)
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
