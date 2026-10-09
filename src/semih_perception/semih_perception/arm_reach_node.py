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

This is a best-effort/demo IK -- our arm's reach is small (~0.4m) and the
joint limits are fairly tight, so many target points will be out of range;
those get clamped to the closest reachable pose rather than rejected outright.
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
    the target was farther than the arm's max reach and got clamped.
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

    cos_interior = (D * D - arm_link2_len**2 - arm_link3_len**2) / (2 * arm_link2_len * arm_link3_len)
    cos_interior = clamp(cos_interior, -1.0, 1.0)
    interior = math.acos(cos_interior)   # angle between link2 and link3, pi = fully extended

    joint3 = -(math.pi - interior)  # elbow bend, 0 = straight, negative = folding "up/back"

    phi_target = math.atan2(r, dz)  # angle from vertical (+Z) toward the target
    alpha = math.atan2(
        arm_link3_len * math.sin(math.pi - interior),
        arm_link2_len + arm_link3_len * math.cos(math.pi - interior),
    )
    joint2 = phi_target - alpha

    joint2 = clamp(joint2, -joint23_limit, joint23_limit)
    joint3 = clamp(joint3, -joint23_limit, joint23_limit)

    return joint1, joint2, joint3, was_clamped


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
        self.declare_parameter('arm_link3_len', 0.15)
        self.declare_parameter('joint1_limit', 3.14)
        self.declare_parameter('joint23_limit', 1.57)

        self.base_height = self.get_parameter('base_height').value
        self.arm_link1_len = self.get_parameter('arm_link1_len').value
        self.arm_link2_len = self.get_parameter('arm_link2_len').value
        self.arm_link3_len = self.get_parameter('arm_link3_len').value
        self.joint1_limit = self.get_parameter('joint1_limit').value
        self.joint23_limit = self.get_parameter('joint23_limit').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.subscription = self.create_subscription(
            PointStamped, '/target_point', self.target_callback, 10
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
        self.get_logger().info('Arm reach node started, waiting for /target_point ...')

    def target_callback(self, msg: PointStamped):
        now = time.time()
        if now - self.last_goal_time < MIN_GOAL_INTERVAL:
            return  # throttle

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
                f'Target ({tx:.2f},{ty:.2f},{tz:.2f}) is out of reach, clamping to max reach.',
                throttle_duration_sec=2.0,
            )

        self.get_logger().info(
            f'Target base_link=({tx:.2f},{ty:.2f},{tz:.2f}) -> '
            f'joints=({math.degrees(joint1):.0f}, {math.degrees(joint2):.0f}, {math.degrees(joint3):.0f}) deg'
        )

        self.send_trajectory(joint1, joint2, joint3)
        self.publish_action(joint1, joint2, joint3)
        self.last_goal_time = now

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
        point = JointTrajectoryPoint()
        point.positions = [joint1, joint2, joint3]
        point.time_from_start = Duration(sec=2, nanosec=0)
        goal.trajectory.points = [point]

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
