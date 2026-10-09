#!/usr/bin/env python3
"""
Synchronized data collection node.

Subscribes to camera image, lidar scan and base odometry using an
approximate-time synchronizer (joint_states and the commanded arm action are
sampled separately -- see the comments near those subscriptions for why),
buffers matched sets in memory, and on shutdown (or after a configured
duration) writes everything into a single HDF5 file.

This is the "핵심 토픽 동기화 수집" MVP described in the original data-team
proposal: arm joint values + camera image + lidar scan + base trajectory,
packed into one HDF5 file with per-topic original timestamps preserved so
sync gaps / frame drops can be checked afterward.
"""

import os
import time
import datetime

import numpy as np
import h5py

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import message_filters
from sensor_msgs.msg import Image, LaserScan, JointState
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge


class Hdf5CollectorNode(Node):
    def __init__(self):
        super().__init__('hdf5_collector_node')

        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('action_topic', '/arm_target_joint_states')
        self.declare_parameter('duration_sec', 10.0)
        self.declare_parameter('output_dir', os.path.expanduser('~/semih_datasets'))
        self.declare_parameter('sync_slop', 0.1)

        camera_topic = self.get_parameter('camera_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        joint_states_topic = self.get_parameter('joint_states_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        action_topic = self.get_parameter('action_topic').value
        self.duration_sec = self.get_parameter('duration_sec').value
        self.output_dir = self.get_parameter('output_dir').value
        sync_slop = self.get_parameter('sync_slop').value

        os.makedirs(self.output_dir, exist_ok=True)

        self.bridge = CvBridge()

        # Buffers (Python lists during collection, converted to numpy at save time)
        self.sync_timestamps = []
        self.images = []
        self.scan_ranges = []
        self.joint_positions = []
        self.joint_velocities = []
        self.joint_names = None
        self.action_positions = []
        self.odom_position = []
        self.odom_orientation = []
        self.odom_linear_vel = []
        self.odom_angular_vel = []

        # Per-topic raw arrival timestamps, for sync-quality checking after the fact
        self.raw_stamps = {'camera': [], 'scan': [], 'joint_states': [], 'odom': []}

        # /joint_states is bridged straight from Gazebo's JointStatePublisher system,
        # which on Fortress (ign-gazebo6) ignores the <update_rate> SDF tag and publishes
        # every physics iteration (measured ~850Hz). Putting a topic that fast into the
        # ApproximateTimeSynchronizer starved it: with queue_size=20, the joint_states
        # deque only spans ~20/850s =~ 0.02s, far under the 0.1s slop, so by the time a
        # ~22Hz camera frame arrived the matching joint_states entry had already been
        # evicted -- this, not QoS, is why 0 frames were ever collected. Joint angles
        # change slowly compared to that noise anyway, so instead of syncing it, just
        # cache the latest value and attach it to whatever the 3-way (camera/scan/odom)
        # sync produces.
        self.latest_joint_state = None
        self.create_subscription(
            JointState, joint_states_topic, self._joint_state_callback, qos_profile_sensor_data
        )

        # Commanded arm target (published by arm_reach_node), sampled the same way as
        # joint_states above -- it's emitted far less often (throttled to ~1/1.5s there)
        # than frames are recorded here, so each frame just carries whatever the most
        # recently commanded target was. This is the "action" label for downstream
        # imitation-learning use (see hdf5_to_lerobot.py); stays None/NaN until the
        # first command of the session.
        self.latest_action = None
        self.create_subscription(JointState, action_topic, self._action_callback, 10)

        # ros_gz_bridge publishes sensor/odom topics as best-effort; reliable subscribers
        # here would silently receive nothing (this is what caused /odom and /scan to look
        # "unresponsive" -- see troubleshooting log #16).
        camera_sub = message_filters.Subscriber(self, Image, camera_topic, qos_profile=qos_profile_sensor_data)
        scan_sub = message_filters.Subscriber(self, LaserScan, scan_topic, qos_profile=qos_profile_sensor_data)
        odom_sub = message_filters.Subscriber(self, Odometry, odom_topic, qos_profile=qos_profile_sensor_data)

        self.sync = message_filters.ApproximateTimeSynchronizer(
            [camera_sub, scan_sub, odom_sub], queue_size=20, slop=sync_slop
        )
        self.sync.registerCallback(self.synced_callback)

        self.start_time = time.time()
        self.timer = self.create_timer(0.5, self.check_duration)

        self.get_logger().info(
            f'Collecting synchronized data for {self.duration_sec:.1f}s '
            f'from [{camera_topic}, {scan_topic}, {joint_states_topic}, {odom_topic}] ...'
        )

    def check_duration(self):
        elapsed = time.time() - self.start_time
        if elapsed >= self.duration_sec:
            self.get_logger().info(f'Duration reached ({elapsed:.1f}s). Saving and shutting down.')
            self.save_and_shutdown()

    def _joint_state_callback(self, msg):
        # Two publishers share this topic: joint_state_broadcaster (ros2_control, arm
        # joints only, ~100Hz) and Gazebo's raw JointStatePublisher bridge (all joints
        # including wheels, ~850Hz -- see the comment above). Their messages have
        # different lengths, and caching whichever arrived last made np.stack() at save
        # time fail on ragged arrays. Keep only the arm-only variant for a fixed shape.
        if 'left_wheel_joint' in msg.name:
            return
        self.latest_joint_state = msg

    def _action_callback(self, msg):
        self.latest_action = msg

    def synced_callback(self, img_msg, scan_msg, odom_msg):
        if self.latest_joint_state is None:
            return  # haven't received a single joint_states message yet, skip

        joint_msg = self.latest_joint_state
        stamp = img_msg.header.stamp.sec + img_msg.header.stamp.nanosec * 1e-9
        self.sync_timestamps.append(stamp)

        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Image conversion failed, skipping frame: {e}')
            self.sync_timestamps.pop()
            return
        self.images.append(cv_image.copy())

        self.scan_ranges.append(np.array(scan_msg.ranges, dtype=np.float32))

        if self.joint_names is None:
            self.joint_names = list(joint_msg.name)
        self.joint_positions.append(np.array(joint_msg.position, dtype=np.float32))
        self.joint_velocities.append(
            np.array(joint_msg.velocity, dtype=np.float32) if joint_msg.velocity else np.array([], dtype=np.float32)
        )

        if self.latest_action is not None:
            self.action_positions.append(np.array(self.latest_action.position, dtype=np.float32))
        else:
            # No command has been sent yet this session (e.g. pure nav/perception
            # run with no arm activity) -- record NaNs rather than skip the frame.
            self.action_positions.append(np.full(len(joint_msg.position), np.nan, dtype=np.float32))

        p = odom_msg.pose.pose.position
        o = odom_msg.pose.pose.orientation
        lv = odom_msg.twist.twist.linear
        av = odom_msg.twist.twist.angular
        self.odom_position.append([p.x, p.y, p.z])
        self.odom_orientation.append([o.x, o.y, o.z, o.w])
        self.odom_linear_vel.append([lv.x, lv.y, lv.z])
        self.odom_angular_vel.append([av.x, av.y, av.z])

        def to_sec(stamp_msg):
            return stamp_msg.sec + stamp_msg.nanosec * 1e-9

        self.raw_stamps['camera'].append(to_sec(img_msg.header.stamp))
        self.raw_stamps['scan'].append(to_sec(scan_msg.header.stamp))
        self.raw_stamps['joint_states'].append(to_sec(joint_msg.header.stamp))
        self.raw_stamps['odom'].append(to_sec(odom_msg.header.stamp))

        n = len(self.sync_timestamps)
        if n % 10 == 0:
            self.get_logger().info(f'Collected {n} synchronized frames so far...')

    def save_and_shutdown(self):
        self.timer.cancel()
        n = len(self.sync_timestamps)

        if n == 0:
            self.get_logger().warn('No synchronized frames were collected. Nothing to save.')
            rclpy.shutdown()
            return

        timestamp_str = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        filepath = os.path.join(self.output_dir, f'semih_session_{timestamp_str}.h5')

        with h5py.File(filepath, 'w') as f:
            f.attrs['num_frames'] = n
            f.attrs['created'] = timestamp_str
            f.attrs['joint_names'] = self.joint_names if self.joint_names else []

            f.create_dataset('sync_timestamps', data=np.array(self.sync_timestamps, dtype=np.float64))

            try:
                f.create_dataset(
                    'camera/images', data=np.stack(self.images), compression='gzip', compression_opts=4
                )
                f.create_dataset('camera/timestamps', data=np.array(self.raw_stamps['camera'], dtype=np.float64))
            except ValueError as e:
                # e.g. camera resolution changed mid-session. Rather than lose every other
                # modality too, skip just the image dataset and keep going.
                self.get_logger().error(f'Could not stack camera images (shape mismatch?), dropping camera/images: {e}')

            # Ragged lidar arrays (should all be same length in practice, but store safely)
            try:
                scan_array = np.stack(self.scan_ranges)
                f.create_dataset('lidar/ranges', data=scan_array, compression='gzip', compression_opts=4)
            except ValueError:
                self.get_logger().warn('Lidar scan lengths varied, storing as variable-length dataset.')
                vlen_dtype = h5py.vlen_dtype(np.dtype('float32'))
                ds = f.create_dataset('lidar/ranges', (n,), dtype=vlen_dtype)
                for i, r in enumerate(self.scan_ranges):
                    ds[i] = r
            f.create_dataset('lidar/timestamps', data=np.array(self.raw_stamps['scan'], dtype=np.float64))

            try:
                f.create_dataset('joint_states/positions', data=np.stack(self.joint_positions))
                f.create_dataset('joint_states/velocities', data=np.stack(self.joint_velocities))
            except ValueError as e:
                self.get_logger().error(f'Could not stack joint states (shape mismatch?), dropping joint_states/*: {e}')
            f.create_dataset('joint_states/timestamps', data=np.array(self.raw_stamps['joint_states'], dtype=np.float64))

            # Commanded arm target per frame (NaN rows where no command had been sent
            # yet -- see _action_callback). Not time-synced separately: it's a
            # sample-and-hold of whatever arm_reach_node last commanded.
            f.create_dataset('action/arm_target_positions', data=np.stack(self.action_positions))

            f.create_dataset('odom/position', data=np.array(self.odom_position, dtype=np.float32))
            f.create_dataset('odom/orientation', data=np.array(self.odom_orientation, dtype=np.float32))
            f.create_dataset('odom/linear_velocity', data=np.array(self.odom_linear_vel, dtype=np.float32))
            f.create_dataset('odom/angular_velocity', data=np.array(self.odom_angular_vel, dtype=np.float32))
            f.create_dataset('odom/timestamps', data=np.array(self.raw_stamps['odom'], dtype=np.float64))

        self.get_logger().info(f'Saved {n} synchronized frames to: {filepath}')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = Hdf5CollectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_and_shutdown()


if __name__ == '__main__':
    main()
