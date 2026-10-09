#!/usr/bin/env python3
"""
Minimal YOLO detection node.
Subscribes to a camera image topic, runs YOLOv8 (pretrained COCO weights) on each frame,
and republishes an annotated image showing bounding boxes + class labels.
"""
import traceback
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO


class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/yolo/image_annotated')
        self.declare_parameter('model_name', 'yolov8n.pt')  # smallest/fastest COCO model
        self.declare_parameter('confidence_threshold', 0.4)
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        model_name = self.get_parameter('model_name').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.get_logger().info(f'Loading YOLO model: {model_name}...')
        self.model = YOLO(model_name)
        self.get_logger().info('YOLO model loaded.')
        self.bridge = CvBridge()
        # ros_gz_bridge publishes sensor topics as best-effort; a reliable subscriber
        # here would silently receive nothing, so match qos_profile_sensor_data.
        self.subscription = self.create_subscription(
            Image, input_topic, self.image_callback, qos_profile_sensor_data
        )
        self.publisher = self.create_publisher(Image, output_topic, 10)
        self.frame_count = 0
        self.get_logger().info(f'Subscribed to {input_topic}, publishing annotated frames to {output_topic}')

    def image_callback(self, msg: Image):
        try:
            # 원본 이미지 인코딩을 유연하게 받아옵니다.
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge input conversion failed: {e}')
            return

        # YOLO 추론 수행
        try:
            results = self.model(cv_image, conf=self.confidence_threshold, verbose=False)
        except Exception as e:
            self.get_logger().error(f'YOLO inference failed, skipping frame: {e}')
            return

        # BGR 이미지 결과 생성 및 메모리 연속성(C-contiguous) 보장
        annotated = results[0].plot()
        annotated = np.ascontiguousarray(annotated, dtype=np.uint8)

        self.frame_count += 1
        num_detections = len(results[0].boxes)
        if self.frame_count % 30 == 0:
            self.get_logger().info(f'Frame {self.frame_count}: {num_detections} detection(s)')

        try:
            # cv_bridge.cv2_to_imgmsg has a known KeyError bug on some numpy/opencv
            # version combinations, so we build the Image message manually instead.
            out_msg = Image()
            out_msg.header = msg.header
            out_msg.height = annotated.shape[0]
            out_msg.width = annotated.shape[1]
            out_msg.encoding = 'bgr8'
            out_msg.is_bigendian = 0
            out_msg.step = annotated.shape[1] * annotated.shape[2]
            out_msg.data = annotated.tobytes()
            self.publisher.publish(out_msg)
        except Exception as e:
            self.get_logger().error(f'Publishing failed: {e}\n{traceback.format_exc()}')


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()