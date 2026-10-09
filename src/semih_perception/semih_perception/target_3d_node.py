#!/usr/bin/env python3
"""
Target 3D coordinate extraction node.

Subscribes to RGB image + depth image (approximately time-synchronized) and
camera intrinsics. Runs YOLO on the RGB frame, then for the highest-confidence
detection, looks up the depth at the bounding box center and converts the
(pixel_u, pixel_v, depth) triple into a 3D point using the standard pinhole
camera model:

    X_opt = (u - cx) * Z / fx
    Y_opt = (v - cy) * Z / fy
    Z_opt = Z   (depth itself)

This gives a point in the "optical frame" convention (X-right, Y-down,
Z-forward). We then convert it into our URDF's camera_link convention
(X-forward, Y-left, Z-up) with the standard REP-103 axis remap:

    x_link = z_opt
    y_link = -x_opt
    z_link = -y_opt

The result is published as a geometry_msgs/PointStamped in the camera_link
frame, ready to be transformed into base_link/map via TF by downstream nodes.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
import message_filters
import numpy as np
from cv_bridge import CvBridge
from ultralytics import YOLO


class Target3DNode(Node):
    def __init__(self):
        super().__init__('target_3d_node')

        self.declare_parameter('rgb_topic', '/camera/image_raw')
        self.declare_parameter('depth_topic', '/depth_camera')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('output_topic', '/target_point')
        self.declare_parameter('model_name', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('camera_frame', 'camera_link')

        rgb_topic = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        output_topic = self.get_parameter('output_topic').value
        model_name = self.get_parameter('model_name').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.camera_frame = self.get_parameter('camera_frame').value

        self.get_logger().info(f'Loading YOLO model: {model_name}...')
        self.model = YOLO(model_name)
        self.get_logger().info('YOLO model loaded.')

        self.bridge = CvBridge()
        self.K = None  # camera intrinsics matrix, filled in once camera_info arrives

        # ros_gz_bridge publishes sensor topics as best-effort; reliable subscribers
        # here would silently receive nothing, so match qos_profile_sensor_data.
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, qos_profile_sensor_data
        )

        # Approximate time sync between RGB and depth (they run at different rates: 30Hz vs 15Hz)
        rgb_sub = message_filters.Subscriber(self, Image, rgb_topic, qos_profile=qos_profile_sensor_data)
        depth_sub = message_filters.Subscriber(self, Image, depth_topic, qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=0.1
        )
        self.sync.registerCallback(self.synced_callback)

        self.publisher = self.create_publisher(PointStamped, output_topic, 10)

        self.frame_count = 0
        self.get_logger().info(
            f'Subscribed to {rgb_topic} + {depth_topic}, publishing target points to {output_topic}'
        )

    def camera_info_callback(self, msg: CameraInfo):
        self.K = np.array(msg.k).reshape(3, 3)

    def synced_callback(self, rgb_msg: Image, depth_msg: Image):
        if self.K is None:
            self.get_logger().warn('No camera_info received yet, skipping frame.', throttle_duration_sec=2.0)
            return

        try:
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'RGB conversion failed: {e}')
            return

        try:
            # Depth image is 32-bit float, meters, one channel
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
        except Exception as e:
            self.get_logger().error(f'Depth conversion failed: {e}')
            return

        try:
            results = self.model(rgb_image, conf=self.confidence_threshold, verbose=False)
        except Exception as e:
            self.get_logger().error(f'YOLO inference failed, skipping frame: {e}')
            return
        boxes = results[0].boxes

        self.frame_count += 1

        if len(boxes) == 0:
            return

        # Pick the highest-confidence detection
        confs = boxes.conf.cpu().numpy()
        best_idx = int(np.argmax(confs))
        best_box = boxes.xyxy.cpu().numpy()[best_idx]  # [x1, y1, x2, y2]
        class_id = int(boxes.cls.cpu().numpy()[best_idx])
        class_name = self.model.names[class_id]
        confidence = float(confs[best_idx])

        u = int((best_box[0] + best_box[2]) / 2.0)
        v = int((best_box[1] + best_box[3]) / 2.0)

        # Guard against out-of-bounds or invalid depth (0, NaN, inf can happen at object edges)
        h, w = depth_image.shape[:2]
        if not (0 <= v < h and 0 <= u < w):
            return
        depth = float(depth_image[v, u])
        if not np.isfinite(depth) or depth <= 0.0:
            self.get_logger().warn(
                f'Invalid depth ({depth}) at detection center, skipping this frame.',
                throttle_duration_sec=2.0,
            )
            return

        fx = self.K[0, 0]
        fy = self.K[1, 1]
        cx = self.K[0, 2]
        cy = self.K[1, 2]

        x_opt = (u - cx) * depth / fx
        y_opt = (v - cy) * depth / fy
        z_opt = depth

        # Convert optical-frame point (X-right, Y-down, Z-forward) into our
        # camera_link convention (X-forward, Y-left, Z-up) -- REP-103 style remap.
        x_link = z_opt
        y_link = -x_opt
        z_link = -y_opt

        msg_out = PointStamped()
        msg_out.header = rgb_msg.header
        msg_out.header.frame_id = self.camera_frame
        msg_out.point.x = x_link
        msg_out.point.y = y_link
        msg_out.point.z = z_link
        self.publisher.publish(msg_out)

        if self.frame_count % 15 == 0:
            self.get_logger().info(
                f'[{class_name} {confidence:.2f}] -> '
                f'camera_link frame: x={x_link:.3f} y={y_link:.3f} z={z_link:.3f} (m)'
            )


def main(args=None):
    rclpy.init(args=args)
    node = Target3DNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
