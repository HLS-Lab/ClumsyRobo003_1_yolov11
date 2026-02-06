#!/usr/bin/env python3
"""
YOLOv11 Object Detection Node for ROS 2.

This node subscribes to camera images, performs object detection using YOLOv11,
and publishes detection results and debug images with bounding boxes.
"""

from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from vision_msgs.msg import (
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)
from std_msgs.msg import Header

from cv_bridge import CvBridge
import numpy as np

# Ultralytics YOLO import
from ultralytics import YOLO


class YoloNode(Node):
    """ROS 2 Node for YOLOv11 object detection."""

    def __init__(self) -> None:
        """Initialize the YOLO detection node."""
        super().__init__('yolo_node')

        # Declare parameters
        self.declare_parameter('model_path', 'yolo11n.pt')
        self.declare_parameter('device', 'cpu')
        self.declare_parameter('conf_threshold', 0.5)

        # Get parameters
        self.model_path: str = self.get_parameter('model_path').get_parameter_value().string_value
        self.device: str = self.get_parameter('device').get_parameter_value().string_value
        self.conf_threshold: float = self.get_parameter('conf_threshold').get_parameter_value().double_value

        self.get_logger().info(f'Loading YOLO model: {self.model_path}')
        self.get_logger().info(f'Device: {self.device}')
        self.get_logger().info(f'Confidence threshold: {self.conf_threshold}')

        # Initialize YOLO model
        try:
            self.model = YOLO(self.model_path)
            # Explicitly set device (cpu for RPi4)
            self.model.to(self.device)
            self.get_logger().info('YOLO model loaded successfully!')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            raise

        # Get class names from the model
        self.class_names: List[str] = self.model.names

        # Initialize CV Bridge for ROS <-> OpenCV conversion
        self.cv_bridge = CvBridge()

        # QoS profile for sensor data (Best Effort for low latency)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscriber: camera image
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            sensor_qos
        )

        # Publisher: detection results (vision_msgs/Detection2DArray)
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/yolo/detections',
            10
        )

        # Publisher: debug image with bounding boxes drawn
        self.debug_image_pub = self.create_publisher(
            Image,
            '/yolo/debug_image',
            10
        )

        self.get_logger().info('YOLO node initialized and ready!')

    def image_callback(self, msg: Image) -> None:
        """
        Process incoming camera images and run YOLO inference.

        Args:
            msg: ROS Image message from camera
        """
        try:
            # Convert ROS Image to OpenCV BGR format
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # Run YOLO inference
        try:
            results = self.model(
                cv_image,
                conf=self.conf_threshold,
                verbose=False  # Suppress console output for performance
            )
        except Exception as e:
            self.get_logger().error(f'YOLO inference failed: {e}')
            return

        # Process results
        result = results[0]  # Get first (and only) result

        # Create Detection2DArray message
        detection_array_msg = self._create_detection_array(result, msg.header)
        self.detection_pub.publish(detection_array_msg)

        # Create and publish debug image using ultralytics' built-in plot()
        # This is more efficient than manual drawing
        debug_image = result.plot()  # Returns BGR numpy array with boxes drawn

        try:
            debug_msg = self.cv_bridge.cv2_to_imgmsg(debug_image, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish debug image: {e}')

    def _create_detection_array(
        self,
        result,
        header: Header
    ) -> Detection2DArray:
        """
        Convert YOLO results to ROS Detection2DArray message.

        Args:
            result: YOLO inference result object
            header: ROS message header with timestamp

        Returns:
            Detection2DArray message with all detected objects
        """
        detection_array = Detection2DArray()
        detection_array.header = header

        # Check if there are any detections
        if result.boxes is None or len(result.boxes) == 0:
            return detection_array

        boxes = result.boxes

        for i in range(len(boxes)):
            detection = Detection2D()
            detection.header = header

            # Get bounding box (xyxy format: x1, y1, x2, y2)
            box = boxes.xyxy[i].cpu().numpy()
            x1, y1, x2, y2 = box

            # Calculate center and size for Detection2D
            # BoundingBox2D uses center point and size
            detection.bbox.center.position.x = float((x1 + x2) / 2.0)
            detection.bbox.center.position.y = float((y1 + y2) / 2.0)
            detection.bbox.size_x = float(x2 - x1)
            detection.bbox.size_y = float(y2 - y1)

            # Get class and confidence
            class_id = int(boxes.cls[i].cpu().numpy())
            confidence = float(boxes.conf[i].cpu().numpy())

            # Create object hypothesis
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(class_id)
            hypothesis.hypothesis.score = confidence

            detection.results.append(hypothesis)

            # Add detection to array
            detection_array.detections.append(detection)

            # Log detection (can be disabled for performance)
            class_name = self.class_names.get(class_id, f'class_{class_id}')
            self.get_logger().debug(
                f'Detected: {class_name} ({confidence:.2f}) at '
                f'[{x1:.0f}, {y1:.0f}, {x2:.0f}, {y2:.0f}]'
            )

        return detection_array


def main(args=None) -> None:
    """Main entry point for the YOLO node."""
    rclpy.init(args=args)

    try:
        node = YoloNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in YOLO node: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
