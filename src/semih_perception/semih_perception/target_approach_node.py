#!/usr/bin/env python3
"""
Target approach node: detect from afar, store the target in odom, then drive to it.

Why: the arm only reaches ~0.30-0.37m (horizontal, from the base center) at the
backpack's height, but YOLO misreads the backpack as "person" (or misses it) once
the camera is that close. So instead of reaching for whatever target_3d_node
publishes at close range, this node

  1. COLLECT: while the robot stands still, takes `lock_samples` consecutive
     /target_point detections, transforms each into `fixed_frame` (odom), and locks
     their median once they all lie within `max_spread` of it.
  2. APPROACH: turns toward the stored point and drives straight at it on /cmd_vel
     (simple P-control on the odom->base_link TF) until the horizontal distance from
     base_link is `standoff`. Detections are ignored from here on.
  3. REACH: stops and publishes the stored point (frame `fixed_frame`) on
     `output_topic` every `republish_period` s; run arm_reach_node with
     input_topic:=/approach/target_point so the arm reaches only now.

The stored point is only as good as odometry: wheel slip between COLLECT and REACH
shows up directly as tip error. Intended for mode:=sim (no Nav2); with Nav2 running,
velocity_smoother also writes /cmd_vel (docs/troubleshooting_p1-p4.md #14).
"""

import math
import statistics
import time
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, Twist
import tf2_ros
from tf2_geometry_msgs import do_transform_point


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


class TargetApproachNode(Node):
    def __init__(self):
        super().__init__('target_approach_node')

        self.declare_parameter('input_topic', '/target_point')
        self.declare_parameter('output_topic', '/approach/target_point')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('fixed_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('lock_samples', 10)
        self.declare_parameter('max_spread', 0.05)
        self.declare_parameter('standoff', 0.33)
        self.declare_parameter('distance_tolerance', 0.01)
        self.declare_parameter('heading_tolerance', 0.05)
        self.declare_parameter('max_linear', 0.12)
        self.declare_parameter('max_angular', 0.4)
        self.declare_parameter('approach_timeout', 90.0)
        self.declare_parameter('republish_period', 2.0)

        p = self.get_parameter
        self.fixed_frame = p('fixed_frame').value
        self.base_frame = p('base_frame').value
        self.lock_samples = p('lock_samples').value
        self.max_spread = p('max_spread').value
        self.standoff = p('standoff').value
        self.dist_tol = p('distance_tolerance').value
        self.heading_tol = p('heading_tolerance').value
        self.max_linear = p('max_linear').value
        self.max_angular = p('max_angular').value
        self.approach_timeout = p('approach_timeout').value
        self.republish_period = p('republish_period').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(PointStamped, p('input_topic').value, self.target_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, p('cmd_vel_topic').value, 10)
        self.out_pub = self.create_publisher(PointStamped, p('output_topic').value, 10)

        self.state = 'COLLECT'
        self.samples = deque(maxlen=self.lock_samples)
        self.target = None  # (x, y, z) in fixed_frame
        self.approach_started = None
        self.last_republish = 0.0
        self.timer = self.create_timer(0.05, self.step)
        self.get_logger().info(
            f'Waiting for {self.lock_samples} stable detections on {p("input_topic").value} ...'
        )

    # ---------------------------------------------------------------- COLLECT
    def target_callback(self, msg: PointStamped):
        if self.state != 'COLLECT':
            return
        try:
            tf = self.tf_buffer.lookup_transform(self.fixed_frame, msg.header.frame_id, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=2.0)
            return
        pt = do_transform_point(msg, tf).point
        self.samples.append((pt.x, pt.y, pt.z))
        if len(self.samples) < self.lock_samples:
            return
        med = tuple(statistics.median(s[i] for s in self.samples) for i in range(3))
        spread = max(math.dist(s, med) for s in self.samples)
        if spread > self.max_spread:
            self.get_logger().info(
                f'Detections not stable yet (spread {spread:.3f} m > {self.max_spread} m).',
                throttle_duration_sec=2.0,
            )
            return
        self.target = med
        self.state = 'APPROACH'
        self.approach_started = time.time()
        self.get_logger().info(
            f'Locked target {self.fixed_frame}=({med[0]:.3f}, {med[1]:.3f}, {med[2]:.3f}) '
            f'from {len(self.samples)} detections (spread {spread:.3f} m). Approaching to '
            f'{self.standoff:.2f} m.'
        )

    # ---------------------------------------------------------------- helpers
    def target_in_base(self):
        msg = PointStamped()
        msg.header.frame_id = self.fixed_frame
        msg.point.x, msg.point.y, msg.point.z = self.target
        tf = self.tf_buffer.lookup_transform(self.base_frame, self.fixed_frame, rclpy.time.Time())
        return do_transform_point(msg, tf).point

    def stop(self):
        self.cmd_pub.publish(Twist())

    # ---------------------------------------------------------------- APPROACH / REACH
    def step(self):
        if self.state == 'APPROACH':
            try:
                pt = self.target_in_base()
            except Exception as e:
                self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=2.0)
                self.stop()
                return
            heading = math.atan2(pt.y, pt.x)
            dist = math.hypot(pt.x, pt.y)
            err = dist - self.standoff
            cmd = Twist()
            if time.time() - self.approach_started > self.approach_timeout:
                self.stop()
                self.state = 'FAILED'
                self.get_logger().error(
                    f'Approach timed out; target base_link=({pt.x:.3f}, {pt.y:.3f}, {pt.z:.3f}).'
                )
                return
            if abs(err) <= self.dist_tol and abs(heading) <= self.heading_tol:
                self.stop()
                self.state = 'REACH'
                self.get_logger().info(
                    f'Arrived: target base_link=({pt.x:.3f}, {pt.y:.3f}, {pt.z:.3f}), '
                    f'horizontal {dist:.3f} m, heading {math.degrees(heading):.1f} deg. '
                    f'Publishing stored target for the arm.'
                )
                return
            cmd.angular.z = clamp(1.5 * heading, -self.max_angular, self.max_angular)
            # Drive only when roughly facing the target, so the base doesn't arc past it.
            if abs(heading) < 3 * self.heading_tol:
                cmd.linear.x = clamp(0.8 * err, -self.max_linear, self.max_linear)
            self.cmd_pub.publish(cmd)
        elif self.state == 'REACH':
            now = time.time()
            if now - self.last_republish >= self.republish_period:
                msg = PointStamped()
                msg.header.frame_id = self.fixed_frame
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.point.x, msg.point.y, msg.point.z = self.target
                self.out_pub.publish(msg)
                self.last_republish = now


def main(args=None):
    rclpy.init(args=args)
    node = TargetApproachNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():  # Ctrl-C already shut the context down in Humble
            rclpy.shutdown()


if __name__ == '__main__':
    main()
