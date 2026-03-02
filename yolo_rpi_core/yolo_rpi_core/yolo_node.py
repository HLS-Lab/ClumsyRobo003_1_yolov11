#!/usr/bin/env python3
"""
YOLOv11 Object Detection Node for ROS 2.

This node subscribes to camera images, performs object detection using YOLOv11,
and publishes detection results and debug images with bounding boxes.

Compatible with: ROS 2 Jazzy, Python 3.12, Ultralytics 8.x+
Target Platform: Raspberry Pi 4 (CPU inference)
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

from cv_bridge import CvBridge

# Ultralytics YOLO - pre-installed in Docker container
from ultralytics import YOLO


class YoloNode(Node):
    """ROS 2 node for YOLOv11 object detection."""

    def __init__(self) -> None:
        """Initialize the YOLO detection node."""
        super().__init__("yolo_node")

        # ---------------------------------------------------------------------
        # Declare Parameters
        # ---------------------------------------------------------------------
        self.declare_parameter("model_path", "yolo11n.pt")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("conf_threshold", 0.5)

        # Get parameter values
        self.model_path = self._get_string_param("model_path")
        self.device = self._get_string_param("device")
        self.conf_threshold = self._get_float_param("conf_threshold")

        self.get_logger().info(f"Loading YOLO model: {self.model_path}")
        self.get_logger().info(f"Device: {self.device}")
        self.get_logger().info(f"Confidence threshold: {self.conf_threshold}")

        # ---------------------------------------------------------------------
        # Initialize YOLO Model
        # ---------------------------------------------------------------------
        try:
            self.model = YOLO(self.model_path)
            # Force model to specified device
            self.model.to(self.device)
            self.get_logger().info(
                f"YOLO model loaded successfully on {self.model.device}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            raise

        # ---------------------------------------------------------------------
        # CV Bridge for ROS <-> OpenCV conversion
        # ---------------------------------------------------------------------
        self.bridge = CvBridge()

        # ---------------------------------------------------------------------
        # QoS Profile - SensorDataQoS for low-latency camera streams
        # ---------------------------------------------------------------------
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # ---------------------------------------------------------------------
        # Subscriber: Camera image input
        # ---------------------------------------------------------------------
        self.image_sub = self.create_subscription(
            Image, "/image_raw", self.image_callback, sensor_qos
        )

        # ---------------------------------------------------------------------
        # Publishers
        # ---------------------------------------------------------------------
        # Detection results
        self.detection_pub = self.create_publisher(
            Detection2DArray, "/yolo/detections", 10
        )

        # Debug image with bounding boxes drawn
        self.debug_image_pub = self.create_publisher(Image, "/yolo/debug_image", 10)

        self.get_logger().info("YoloNode initialized and ready!")

    def _get_string_param(self, name: str) -> str:
        return self.get_parameter(name).get_parameter_value().string_value

    def _get_float_param(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    def image_callback(self, msg: Image) -> None:
        """
        Process incoming camera image and run YOLO inference.

        Args:
            msg: ROS Image message from camera
        """
        try:
            # Convert ROS Image to OpenCV BGR format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Run YOLO inference
        try:
            results = self.model(cv_image, verbose=False, conf=self.conf_threshold)
        except Exception as e:
            self.get_logger().error(f"YOLO inference failed: {e}")
            return

        # Process results
        if results and len(results) > 0:
            result = results[0]

            # Generate and publish Detection2DArray
            detection_array = self._create_detection_array(result, msg.header)
            self.detection_pub.publish(detection_array)

            # Generate and publish debug image using ultralytics built-in plot()
            # This is more efficient than manual drawing
            debug_cv_image = result.plot()

            try:
                debug_msg = self.bridge.cv2_to_imgmsg(debug_cv_image, encoding="bgr8")
                debug_msg.header = msg.header
                self.debug_image_pub.publish(debug_msg)
            except Exception as e:
                self.get_logger().error(f"Failed to publish debug image: {e}")

    def _create_detection_array(self, result, header) -> Detection2DArray:
        """
        Convert YOLO results to ROS Detection2DArray message.

        Args:
            result: YOLO inference result object
            header: ROS message header from source image

        Returns:
            Detection2DArray message with all detections
        """
        detection_array = Detection2DArray()
        detection_array.header = header

        # Get class names from model
        class_names: dict = self.model.names

        # Check if there are any boxes
        if result.boxes is None or len(result.boxes) == 0:
            return detection_array

        # Process each detection
        for box in result.boxes:
            detection = Detection2D()
            detection.header = header

            # Get bounding box coordinates (xyxy format)
            xyxy = box.xyxy[0].cpu().numpy()
            x1, y1, x2, y2 = xyxy

            # Calculate center and size for Detection2D
            center_x = (x1 + x2) / 2.0
            center_y = (y1 + y2) / 2.0
            width = x2 - x1
            height = y2 - y1

            # Set bounding box
            detection.bbox.center.position.x = float(center_x)
            detection.bbox.center.position.y = float(center_y)
            detection.bbox.size_x = float(width)
            detection.bbox.size_y = float(height)

            # Set detection hypothesis
            hypothesis = ObjectHypothesisWithPose()

            # Get class ID and confidence
            class_id = int(box.cls[0].item())
            confidence = float(box.conf[0].item())

            # Map class ID to class name
            class_name = class_names.get(class_id, f"class_{class_id}")
            hypothesis.hypothesis.class_id = class_name
            hypothesis.hypothesis.score = confidence

            detection.results.append(hypothesis)
            detection_array.detections.append(detection)

        return detection_array


def main(args: Optional[List[str]] = None) -> None:
    """Entry point for the YOLO node."""
    rclpy.init(args=args)
    node: Optional[YoloNode] = None

    try:
        node = YoloNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"YoloNode error: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
