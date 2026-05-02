#!/usr/bin/env python3
"""go_to_color_ball — Phase 2 demo.

Subscribes to:
  /target_color        std_msgs/String          ("red", "yellow", "blue")
  /yolo/detections     yolo_msgs/DetectionArray
  /camera/image        sensor_msgs/Image
  /camera/camera_info  sensor_msgs/CameraInfo

Publishes:
  /goal_pose           geometry_msgs/PoseStamped     (Nav2 NavigateToPose entry)

For each YOLO "sports ball" detection, samples the bbox-center pixels in the
most recent camera image, classifies the ball as red/yellow/blue from hue,
and if the color matches the requested target_color, projects the bbox
bottom-center to the ground plane via the camera intrinsics + map ← optical
TF, then publishes a Nav2 goal `stop_distance` metres in front of the ball
with yaw facing it.

Trigger from the CLI:
  ros2 topic pub --once /target_color std_msgs/String "{data: red}"
"""

import math
import threading

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from yolo_msgs.msg import DetectionArray

from cv_bridge import CvBridge
import tf2_ros


# Hue ranges in OpenCV HSV (H ∈ [0, 179]). Saturation gate excludes greys.
COLORS = {
    'red':    lambda h, s: s > 40 and (h < 10 or h > 170),
    'yellow': lambda h, s: s > 40 and 15 <= h < 35,
    'blue':   lambda h, s: s > 40 and 95 <= h < 130,
}


class GoToColorBall(Node):
    def __init__(self):
        super().__init__('go_to_color_ball')
        self.declare_parameter('target_class',     'sports ball')
        self.declare_parameter('stop_distance',    0.5)
        self.declare_parameter('map_frame',        'map')
        self.declare_parameter('optical_frame',    'sim_camera_optical_frame')
        self.declare_parameter('base_frame',       'base_footprint')
        self.declare_parameter('republish_period', 3.0)
        self.declare_parameter('min_score',        0.3)

        self.bridge       = CvBridge()
        self.tf_buffer    = tf2_ros.Buffer()
        self.tf_listener  = tf2_ros.TransformListener(self.tf_buffer, self)

        self.K            = None
        self.image_width  = None    # from camera_info
        self.last_image   = None
        self.target_color = None
        self.lock         = threading.Lock()
        self.last_goal    = None
        self.last_goal_t  = self.get_clock().now()
        self.last_match_t = None    # time of last matching ball detection
        # Fine-adjust mode: after Nav2 arrival, center the ball in the camera
        # view via pixel-error-driven yaw nudges. Pixel center within ±CENTER_PX
        # of image center counts as "in front of camera".
        self.fine_adjust         = False
        self.fine_attempts       = 0      # how many corrections we've sent this round
        self.fine_started_t      = None
        self.CENTER_PX           = 25     # ≈ ±8° in a 320-px-wide 60° FOV image
        self.MAX_FINE_ATTEMPTS   = 4      # cap; converging within Nav2 yaw tol is unlikely past this
        self.FINE_LOST_TIMEOUT   = 6.0    # s — if YOLO loses the ball this long while fine-adjusting,
                                          #     assume we're too close (frame full of ball) and stop

        # Search behaviour: when target_color is set but no matching ball is
        # visible, the robot spins in place to scan ~360°, then drives to the
        # next room centre and scans again. Coordinates are in map frame; the
        # current AMCL set_initial_pose puts the robot at map (-1.5, 0) on
        # spawn, so map ≡ world here.
        self.SEARCH_WAYPOINTS = [
            (-2.0,  0.0),    # living-room centre
            ( 0.0,  0.0),    # central hallway
            ( 1.5,  1.5),    # bedroom
            ( 0.0,  0.0),    # back through hallway
            ( 1.5, -1.5),    # kitchen
        ]
        self.search_step = 0    # 0..5 = spin 60° each, 6 = drive to next room
        self.room_idx    = 0

        # "Navigating to a ball" lock: once we publish a goal in response to a
        # detection, suppress the search timer until the robot actually arrives
        # at that goal (or a long timeout). Without this lock, the search timer
        # fires every 5 s and preempts the navigate-to-ball goal whenever the
        # ball briefly drops out of the camera (which it does as the robot
        # turns + closes distance).
        self.navigating       = False
        self.nav_goal_xy      = None
        self.nav_started_t    = None
        self.NAV_ARRIVAL_TOL  = 0.40   # m  — close enough to consider arrived
        self.NAV_TIMEOUT      = 60.0   # s  — abandon a stuck nav after this

        self._search_timer = self.create_timer(5.0, self._search_tick)

        self.create_subscription(String,         '/target_color',       self._on_target,     10)
        self.create_subscription(CameraInfo,     '/camera/camera_info', self._on_caminfo,    10)
        self.create_subscription(Image,          '/camera/image',       self._on_image,      10)
        self.create_subscription(DetectionArray, '/yolo/detections',    self._on_detections, 10)

        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.get_logger().info(
            'ready. set target with: '
            'ros2 topic pub --once /target_color std_msgs/String "{data: red}"'
        )

    # ---------------- callbacks ----------------

    def _on_target(self, msg):
        c = msg.data.strip().lower()
        if c not in COLORS:
            self.get_logger().warn(f'unknown color "{c}" — expected one of {list(COLORS)}')
            return
        with self.lock:
            self.target_color  = c
            self.last_goal     = None    # allow immediate re-publish
            self.last_match_t  = None    # reset match tracking
            self.search_step   = 0       # restart the scan from spin
            self.navigating    = False   # release any prior nav lock
            self.nav_goal_xy   = None
            self.fine_adjust   = False
        self.get_logger().info(f'target color set to: {c} — searching')

    def _on_caminfo(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.image_width = msg.width

    def _on_image(self, msg):
        with self.lock:
            self.last_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def _on_detections(self, msg):
        with self.lock:
            target = self.target_color
            img    = self.last_image
            K      = self.K
        if target is None or img is None or K is None:
            return

        target_class = self.get_parameter('target_class').value
        min_score    = self.get_parameter('min_score').value

        best = None  # (score, det)
        for det in msg.detections:
            if det.class_name != target_class or det.score < min_score:
                continue
            color = self._sample_color(img, det.bbox)
            if color != target:
                continue
            if best is None or det.score > best[0]:
                best = (det.score, det)

        if best is None:
            return

        det = best[1]
        self.last_match_t = self.get_clock().now()    # cancels search loop

        # ---- fine-adjust path: ball is in view AFTER arrival, center it ----
        if self.fine_adjust and self.image_width is not None:
            self._fine_adjust(det)
            return

        u = det.bbox.center.position.x
        v = det.bbox.center.position.y + det.bbox.size.y / 2.0   # bottom-center

        try:
            ox, oy = self._project_to_ground(u, v)
            rx, ry = self._robot_xy()
        except Exception as e:
            self.get_logger().warn(f'projection/TF failed: {e}', throttle_duration_sec=2.0)
            return

        dx, dy = ox - rx, oy - ry
        dist   = math.hypot(dx, dy)
        stop_d = self.get_parameter('stop_distance').value
        if dist < stop_d:
            self.get_logger().info(f'already within {stop_d:.2f} m of {target} ball')
            return
        gx  = ox - stop_d * dx / dist
        gy  = oy - stop_d * dy / dist
        yaw = math.atan2(dy, dx)

        # Throttle re-publishes to keep Nav2 from preempting itself
        now = self.get_clock().now()
        if self.last_goal is not None:
            ddx = gx - self.last_goal[0]
            ddy = gy - self.last_goal[1]
            dt  = (now - self.last_goal_t).nanoseconds / 1e9
            if math.hypot(ddx, ddy) < 0.1 and dt < self.get_parameter('republish_period').value:
                return

        out = PoseStamped()
        out.header.stamp    = now.to_msg()
        out.header.frame_id = self.get_parameter('map_frame').value
        out.pose.position.x       = gx
        out.pose.position.y       = gy
        out.pose.orientation.z    = math.sin(yaw / 2)
        out.pose.orientation.w    = math.cos(yaw / 2)
        self.goal_pub.publish(out)
        self.last_goal     = (gx, gy)
        self.last_goal_t   = now
        self.navigating    = True
        self.nav_goal_xy   = (gx, gy)
        self.nav_started_t = now
        self.get_logger().info(
            f'{target} ball @ ({ox:+.2f},{oy:+.2f}) → goal ({gx:+.2f},{gy:+.2f}) '
            f'yaw {math.degrees(yaw):+.0f}° (locked from search)'
        )

    # ---------------- helpers ----------------

    def _sample_color(self, img, bbox):
        h_img, w_img = img.shape[:2]
        cx, cy = int(bbox.center.position.x), int(bbox.center.position.y)
        r = max(int(min(bbox.size.x, bbox.size.y) * 0.25), 2)
        x0, x1 = max(0, cx - r), min(w_img, cx + r + 1)
        y0, y1 = max(0, cy - r), min(h_img, cy + r + 1)
        if x0 >= x1 or y0 >= y1:
            return None
        hsv = cv2.cvtColor(img[y0:y1, x0:x1], cv2.COLOR_BGR2HSV)
        h_med = float(np.median(hsv[:, :, 0]))
        s_med = float(np.median(hsv[:, :, 1]))
        for color, pred in COLORS.items():
            if pred(h_med, s_med):
                return color
        return None

    def _project_to_ground(self, u, v):
        K = self.K
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        ray = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
        ray /= np.linalg.norm(ray)

        t = self.tf_buffer.lookup_transform(
            self.get_parameter('map_frame').value,
            self.get_parameter('optical_frame').value, Time())
        tx, ty, tz = t.transform.translation.x, t.transform.translation.y, t.transform.translation.z
        q = t.transform.rotation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        R = np.array([
            [1 - 2*(qy*qy + qz*qz),  2*(qx*qy - qz*qw),      2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw),      1 - 2*(qx*qx + qz*qz),  2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw),      2*(qy*qz + qx*qw),      1 - 2*(qx*qx + qy*qy)],
        ])
        ray_map = R @ ray
        if abs(ray_map[2]) < 1e-6:
            raise ValueError('ray parallel to ground')
        t_param = -tz / ray_map[2]
        if t_param <= 0:
            raise ValueError('object behind camera or above horizon')
        return float(tx + t_param * ray_map[0]), float(ty + t_param * ray_map[1])

    def _robot_xy(self):
        t = self.tf_buffer.lookup_transform(
            self.get_parameter('map_frame').value,
            self.get_parameter('base_frame').value, Time())
        return float(t.transform.translation.x), float(t.transform.translation.y)

    def _robot_yaw(self):
        t = self.tf_buffer.lookup_transform(
            self.get_parameter('map_frame').value,
            self.get_parameter('base_frame').value, Time())
        q = t.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    def _fine_adjust(self, det):
        """Center the detected ball in the camera frame by yaw correction.
        Done when bbox center pixel is within ±CENTER_PX of image-center, OR
        we've made MAX_FINE_ATTEMPTS without converging (Nav2's yaw goal
        tolerance can absorb small corrections, leading to no actual rotation
        and an endless detection→goal loop). Holds robot position; only rotates."""
        u = det.bbox.center.position.x
        offset_px = u - (self.image_width / 2.0)

        if abs(offset_px) <= self.CENTER_PX:
            self._finish(f'ball centered (offset {offset_px:+.0f}px)')
            return

        if self.fine_attempts >= self.MAX_FINE_ATTEMPTS:
            self._finish(
                f'ball off-center by {offset_px:+.0f}px after '
                f'{self.fine_attempts} fine-adjust attempts; stopping'
            )
            return

        try:
            rx, ry = self._robot_xy()
            ryaw   = self._robot_yaw()
        except Exception as e:
            self.get_logger().warn(f'TF: {e}', throttle_duration_sec=2.0)
            return

        # P-controller: positive offset_px = ball is right-of-image-center in
        # optical convention → robot should turn RIGHT (negative yaw).
        # Half-image-width maps to ~30° rotation.
        norm = offset_px / (self.image_width / 2.0)
        yaw_correction = -norm * math.radians(30)
        new_yaw = ryaw + yaw_correction
        self.fine_attempts += 1
        self._publish_search_goal(
            rx, ry, new_yaw,
            label=f'fine-adjust {self.fine_attempts}/{self.MAX_FINE_ATTEMPTS} '
                  f'(offset {offset_px:+.0f}px → Δyaw {math.degrees(yaw_correction):+.0f}°)'
        )

    def _finish(self, reason):
        """Mark the run done — clears target color, exits fine-adjust, idles."""
        self.fine_adjust = False
        self.get_logger().info(f'{reason} — done')
        with self.lock:
            self.target_color = None

    def _publish_search_goal(self, gx, gy, yaw, label):
        """Publish a goal for the search loop. Bypasses the navigate-to-ball
        throttle by clearing last_goal so subsequent detections can preempt."""
        out = PoseStamped()
        out.header.stamp    = self.get_clock().now().to_msg()
        out.header.frame_id = self.get_parameter('map_frame').value
        out.pose.position.x = gx
        out.pose.position.y = gy
        out.pose.orientation.z = math.sin(yaw / 2)
        out.pose.orientation.w = math.cos(yaw / 2)
        self.goal_pub.publish(out)
        self.get_logger().info(f'search: {label} → ({gx:+.2f},{gy:+.2f}) yaw {math.degrees(yaw):+.0f}°')

    def _search_tick(self):
        """Periodic search step. Skipped if no target is set, or if we're
        currently navigating to a previously-seen ball."""
        with self.lock:
            target = self.target_color
        if target is None:
            return

        try:
            rx, ry  = self._robot_xy()
            ryaw    = self._robot_yaw()
        except Exception as e:
            self.get_logger().warn(f'TF not ready: {e}', throttle_duration_sec=5.0)
            return

        # If we're locked on a ball goal, check arrival / timeout. Don't run
        # search until that goal is consumed.
        if self.navigating and self.nav_goal_xy is not None:
            gx, gy   = self.nav_goal_xy
            dist     = math.hypot(rx - gx, ry - gy)
            elapsed  = (self.get_clock().now() - self.nav_started_t).nanoseconds / 1e9
            if dist < self.NAV_ARRIVAL_TOL:
                # Reached the Nav2 goal. Switch to fine-adjust: spin to find
                # the ball if not in view, then yaw-correct via pixel error.
                self.navigating       = False
                self.fine_adjust      = True
                self.fine_attempts    = 0
                self.fine_started_t   = self.get_clock().now()
                self.last_match_t     = None    # force a fresh detection check
                self.get_logger().info(f'arrived at {target} ball — fine-adjusting')
                return
            if elapsed > self.NAV_TIMEOUT:
                self.get_logger().warn(
                    f'nav-to-{target}-ball timed out after {elapsed:.0f}s — resuming search'
                )
                self.navigating = False
                self.nav_goal_xy = None
            else:
                return    # still navigating, suppress search

        # In fine-adjust mode without a fresh detection: either the camera
        # missed it briefly (slow-spin to recover) OR we got so close YOLO
        # can't classify it anymore (ball fills frame). After FINE_LOST_TIMEOUT
        # of no detections, assume the latter and call it done.
        if self.fine_adjust:
            now = self.get_clock().now()
            if self.last_match_t is not None:
                dt = (now - self.last_match_t).nanoseconds / 1e9
                if dt < 3.0:
                    return    # ball was just seen; _on_detections is centering
                if dt > self.FINE_LOST_TIMEOUT:
                    self._finish('lost ball at close range (probably right in front)')
                    return
            else:
                # Never got a detection in fine-adjust; bail similarly after timeout
                if self.fine_started_t is not None:
                    elapsed = (now - self.fine_started_t).nanoseconds / 1e9
                    if elapsed > self.FINE_LOST_TIMEOUT:
                        self._finish('no detection at arrival — stopping anyway')
                        return
            self._publish_search_goal(rx, ry, ryaw + math.radians(45),
                                       label='fine-spin (ball not in view)')
            return

        # Steps 0..5 = spin 60° each (≈360° total); step 6 = drive to next
        # waypoint then start spinning again.
        if self.search_step < 6:
            tgt_yaw = ryaw + math.radians(60)
            self._publish_search_goal(rx, ry, tgt_yaw,
                                       label=f'spin {self.search_step + 1}/6')
        else:
            wx, wy = self.SEARCH_WAYPOINTS[self.room_idx]
            yaw    = math.atan2(wy - ry, wx - rx)
            self._publish_search_goal(wx, wy, yaw,
                                       label=f'waypoint {self.room_idx + 1}/{len(self.SEARCH_WAYPOINTS)}')
            self.room_idx = (self.room_idx + 1) % len(self.SEARCH_WAYPOINTS)
        self.search_step = (self.search_step + 1) % 7


def main():
    rclpy.init()
    node = GoToColorBall()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
