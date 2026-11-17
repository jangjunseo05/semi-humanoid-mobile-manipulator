#!/usr/bin/env python3
"""
Arm reach node.

Subscribes to /target_point (PointStamped, published by target_3d_node in the
camera_link frame). Transforms it into base_link frame via TF, then solves a
simple analytical inverse kinematics problem for our 3-DOF arm:

  - arm_joint1 (yaw, at the base): points the arm toward the target's
    horizontal direction.
  - arm_joint2 + arm_joint3 (pitch, "shoulder" + "elbow"): a classic 2-link
    planar arm IK, solved in the vertical plane after the yaw rotation.

The resulting joint angles are sent as a FollowJointTrajectory goal to the
arm_controller action server (the same one we tested manually earlier).

This is a best-effort/demo IK -- our arm's reach is small (~0.46m) and the
joint limits are fairly tight, so many target points will be out of range;
those get clamped to the closest reachable pose rather than rejected outright.

The input topic is a parameter (`input_topic`, default /target_point) so the arm can
instead follow target_approach_node's stored target (/approach/target_point), which
is only published once the base has driven to a reachable standoff.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import tf2_ros
from tf2_geometry_msgs import do_transform_point


# Minimum time between sending new trajectory goals (seconds), to avoid
# flooding the action server with a new goal on every single camera frame.
MIN_GOAL_INTERVAL = 1.5


def clamp(value, lo, hi):
    return max(lo, min(hi, value))


def solve_ik(tx, ty, tz, *, base_height, arm_link1_len, arm_link2_len, arm_link3_len,
             joint1_limit, joint23_limit):
    """Analytical IK for the 3-DOF arm. Pure function (no ROS/Node dependency)
    so it can be unit-tested directly -- see test/test_arm_ik.py.

    Returns (joint1, joint2, joint3, was_clamped), where was_clamped is True if
    the target was farther than the arm's max reach, or the solution needed a joint
    beyond its limit (e.g. a target below the lowest point the elbow-down pose
    reaches); in both cases the commanded pose does not reach the target.
    """
    # 1) Yaw: horizontal direction to the target.
    joint1 = math.atan2(ty, tx)
    joint1 = clamp(joint1, -joint1_limit, joint1_limit)

    # 2) Planar 2-link IK in the vertical plane, relative to the "shoulder"
    #    point (top of arm_link1, which is always directly above the base
    #    joint since arm_link1 only ever points straight up).
    shoulder_z = base_height / 2.0 + arm_link1_len
    r = math.sqrt(tx * tx + ty * ty)      # horizontal distance from base
    dz = tz - shoulder_z                   # height relative to shoulder

    D = math.sqrt(r * r + dz * dz)
    max_reach = arm_link2_len + arm_link3_len
    was_clamped = D > max_reach
    if was_clamped:
        D = max_reach

    # Law of cosines gives the elbow *bend* directly (0 = fully extended, not the angle
    # between the links). The previous version treated this value as the inter-link angle
    # and then used -(pi - angle) for joint3, so the tip missed every target by ~30cm
    # (caught by forward kinematics in test/test_arm_ik.py::test_tip_reaches_target).
    cos_bend = (D * D - arm_link2_len**2 - arm_link3_len**2) / (2 * arm_link2_len * arm_link3_len)
    bend = math.acos(clamp(cos_bend, -1.0, 1.0))

    # Positive bend = elbow-down solution, which keeps joint2 within its +-90 deg limit
    # for targets in front of the robot.
    joint3 = bend

    phi_target = math.atan2(r, dz)  # angle from vertical (+Z) toward the target
    alpha = math.atan2(
        arm_link3_len * math.sin(bend),
        arm_link2_len + arm_link3_len * math.cos(bend),
    )
    joint2 = phi_target - alpha

    # Before this check, a target within max_reach but outside the joint limits was
    # silently clamped and reported as reachable (the arm stopped short with no warning).
    if abs(joint2) > joint23_limit or abs(joint3) > joint23_limit:
        was_clamped = True
    joint2 = clamp(joint2, -joint23_limit, joint23_limit)
    joint3 = clamp(joint3, -joint23_limit, joint23_limit)

    return joint1, joint2, joint3, was_clamped


def fold_lower_extend(joint1, joint2, joint3, joint23_limit):
    """Waypoints that keep the forearm pointing down/back until the shoulder is lowered.

    A single trajectory point makes the controller interpolate all joints together; for a
    low target in front (e.g. base_link (0.34, 0, 0.09)) the tip then swings out to
    x=0.42m while still ~0.3m up and lands on top of the object instead of reaching its
    near face (seen in Gazebo with the backpack, 2026-11-17). Folding the elbow to its
    limit first caps the tip at sqrt(L2^2 + L3^2) = 0.33m forward while the shoulder
    comes down, and the last leg moves the tip forward/up into the target.
    """
    fold = joint23_limit if joint3 >= 0 else -joint23_limit
    return [(joint1, 0.0, fold), (joint1, joint2, fold), (joint1, joint2, joint3)]


def _same_point(a, b, tol=0.01):
    return (a.header.frame_id == b.header.frame_id
            and abs(a.point.x - b.point.x) < tol
            and abs(a.point.y - b.point.y) < tol
            and abs(a.point.z - b.point.z) < tol)


class ArmReachNode(Node):
    def __init__(self):
        super().__init__('arm_reach_node')

        # Arm geometry -- must match semih.urdf.xacro's properties of the same
        # name (arm_link1_len etc.). Declared as parameters (rather than module
        # constants) so they can be overridden from a launch file if the URDF
        # geometry changes, without editing this file.
        self.declare_parameter('base_height', 0.15)
        self.declare_parameter('arm_link1_len', 0.20)
        self.declare_parameter('arm_link2_len', 0.25)
        self.declare_parameter('arm_link3_len', 0.21)
        self.declare_parameter('joint1_limit', 3.14)
        self.declare_parameter('joint23_limit', 1.57)
        self.declare_parameter('input_topic', '/target_point')
        # Move as fold elbow -> lower shoulder -> extend instead of one joint-space
        # interpolation; see fold_lower_extend(). Off by default because with a live
        # /target_point stream every new goal would fold the arm again.
        self.declare_parameter('fold_first', False)
        self.fold_first = self.get_parameter('fold_first').value
        self.last_target_msg = None

        self.base_height = self.get_parameter('base_height').value
        self.arm_link1_len = self.get_parameter('arm_link1_len').value
        self.arm_link2_len = self.get_parameter('arm_link2_len').value
        self.arm_link3_len = self.get_parameter('arm_link3_len').value
        self.joint1_limit = self.get_parameter('joint1_limit').value
        self.joint23_limit = self.get_parameter('joint23_limit').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        input_topic = self.get_parameter('input_topic').value
        self.subscription = self.create_subscription(
            PointStamped, input_topic, self.target_callback, 10
        )

        self.action_client = ActionClient(
            self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory'
        )

        # Publishes whatever joint angles we just *commanded* (as opposed to
        # /joint_states, which reports what the arm actually *reports*). This is
        # what a data-collection pipeline needs as the "action" label -- see
        # semih_data_collection/hdf5_collector_node.py's action_topic parameter.
        self.action_pub = self.create_publisher(JointState, '/arm_target_joint_states', 10)

        self.last_goal_time = 0.0
        self.get_logger().info(f'Arm reach node started, waiting for {input_topic} ...')

    def target_callback(self, msg: PointStamped):
        now = time.time()
        if now - self.last_goal_time < MIN_GOAL_INTERVAL:
            return  # throttle
        if self.last_target_msg is not None and _same_point(msg, self.last_target_msg):
            return  # e.g. target_approach_node republishing the stored target

        # Transform the point from camera_link into base_link frame via TF.
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link', msg.header.frame_id, rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=2.0)
            return

        point_base = do_transform_point(msg, transform)
        tx = point_base.point.x
        ty = point_base.point.y
        tz = point_base.point.z

        joint1, joint2, joint3, was_clamped = solve_ik(
            tx, ty, tz,
            base_height=self.base_height,
            arm_link1_len=self.arm_link1_len,
            arm_link2_len=self.arm_link2_len,
            arm_link3_len=self.arm_link3_len,
            joint1_limit=self.joint1_limit,
            joint23_limit=self.joint23_limit,
        )
        if was_clamped:
            self.get_logger().warn(
                f'Target ({tx:.2f},{ty:.2f},{tz:.2f}) is out of reach, clamping to the nearest pose.',
                throttle_duration_sec=2.0,
            )

        self.get_logger().info(
            f'Target base_link=({tx:.2f},{ty:.2f},{tz:.2f}) -> '
            f'joints=({math.degrees(joint1):.0f}, {math.degrees(joint2):.0f}, {math.degrees(joint3):.0f}) deg'
        )

        self.send_trajectory(joint1, joint2, joint3)
        self.publish_action(joint1, joint2, joint3)
        self.last_goal_time = now
        self.last_target_msg = msg

    def publish_action(self, joint1, joint2, joint3):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['arm_joint1', 'arm_joint2', 'arm_joint3']
        msg.position = [joint1, joint2, joint3]
        self.action_pub.publish(msg)

    def send_trajectory(self, joint1, joint2, joint3):
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('arm_controller action server not available.')
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['arm_joint1', 'arm_joint2', 'arm_joint3']
        if self.fold_first:
            waypoints = fold_lower_extend(joint1, joint2, joint3, self.joint23_limit)
        else:
            waypoints = [(joint1, joint2, joint3)]
        for i, positions in enumerate(waypoints, start=1):
            point = JointTrajectoryPoint()
            point.positions = list(positions)
            point.time_from_start = Duration(sec=2 * i, nanosec=0)
            goal.trajectory.points.append(point)

        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Trajectory goal was rejected by arm_controller.')


def main(args=None):
    rclpy.init(args=args)
    node = ArmReachNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
