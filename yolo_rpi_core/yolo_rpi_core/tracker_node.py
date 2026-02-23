#!/usr/bin/env python3
"""
Object Tracking Node for ROS 2.

This node subscribes to YOLO detection results and performs frame-to-frame
object tracking. It publishes tracking commands (error offsets from image
center) that downstream actuator nodes can consume to physically follow
the target.

Supported Tracking Algorithms (selected via ``tracker_type`` parameter):
    - ``centroid``  — Greedy centroid-distance matching. Simple, fast.
    - ``bytetrack`` — Kalman filter + IoU matching with two-stage
      association (high-conf then low-conf detections). Better occlusion
      handling. No re-ID network required.

Both algorithms are CPU-friendly (Raspberry Pi 4 compatible).

Compatible with: ROS 2 Jazzy, Python 3.12
Target Platform: Raspberry Pi 4 (CPU inference, NO CUDA)
"""

from __future__ import annotations

import json
import math
from collections import OrderedDict
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

from cv_bridge import CvBridge


# =============================================================================
# Data Classes
# =============================================================================

@dataclass
class TrackedObject:
    """
    Represents a single tracked object across frames.

    Attributes:
        object_id:    Unique tracking ID assigned by the tracker.
        centroid:     (cx, cy) pixel coordinates of the object center.
        bbox:         (x1, y1, x2, y2) bounding box in pixel coordinates.
        class_name:   YOLO class label (e.g. 'person', 'cup').
        confidence:   Detection confidence score [0.0, 1.0].
        disappeared:  Number of consecutive frames without a match.
        velocity:     Estimated velocity (dx, dy) in pixels/frame.
    """
    object_id: int
    centroid: Tuple[float, float]
    bbox: Tuple[float, float, float, float]
    class_name: str = ""
    confidence: float = 0.0
    disappeared: int = 0
    velocity: Tuple[float, float] = (0.0, 0.0)


# =============================================================================
# CentroidTracker — Pure Python, no ROS dependency
# =============================================================================

class CentroidTracker:
    """
    Lightweight centroid-based multi-object tracker.

    Algorithm:
        1. For each new frame, compute centroids of all incoming detections.
        2. If there are existing tracked objects, compute pairwise Euclidean
           distances between existing centroids and new centroids.
        3. Use a greedy assignment (Hungarian-approximation via sorted distances)
           to match new detections to existing objects.
        4. Unmatched existing objects increment their `disappeared` counter.
        5. Objects exceeding `max_disappeared` frames are deregistered.
        6. Unmatched new detections are registered as new objects.

    This tracker is CPU-friendly and suitable for Raspberry Pi 4.

    Args:
        max_disappeared: Number of consecutive frames an object can be
                         missing before it is deregistered.  Default: 30.
        max_distance:    Maximum centroid distance (pixels) to consider
                         a match.  Detections further than this are registered
                         as new objects.  Default: 80.0.
    """

    def __init__(
        self,
        max_disappeared: int = 30,
        max_distance: float = 80.0,
    ) -> None:
        self._next_object_id: int = 0
        self._objects: OrderedDict[int, TrackedObject] = OrderedDict()
        self.max_disappeared: int = max_disappeared
        self.max_distance: float = max_distance

    # -----------------------------------------------------------------
    # Public API
    # -----------------------------------------------------------------

    def update(
        self,
        detections: List[Tuple[Tuple[float, float], Tuple[float, float, float, float], str, float]],
    ) -> Dict[int, TrackedObject]:
        """
        Update tracker state with a new set of detections.

        Args:
            detections: List of tuples, each containing:
                - centroid: (cx, cy) center of bounding box
                - bbox:     (x1, y1, x2, y2) bounding box
                - class_name: YOLO class label string
                - confidence: detection confidence float

        Returns:
            Dictionary mapping object_id → TrackedObject for all currently
            tracked objects.
        """
        # Case 1: No detections — increment disappeared for all objects
        if len(detections) == 0:
            for obj_id in list(self._objects.keys()):
                self._objects[obj_id].disappeared += 1
                if self._objects[obj_id].disappeared > self.max_disappeared:
                    self._deregister(obj_id)
            return dict(self._objects)

        # Extract input centroids as numpy array for vectorized distance calc
        input_centroids = np.array([d[0] for d in detections], dtype=np.float64)

        # Case 2: No existing objects — register all detections
        if len(self._objects) == 0:
            for i in range(len(detections)):
                centroid, bbox, class_name, confidence = detections[i]
                self._register(centroid, bbox, class_name, confidence)
            return dict(self._objects)

        # Case 3: Match existing objects to new detections
        object_ids = list(self._objects.keys())
        object_centroids = np.array(
            [self._objects[oid].centroid for oid in object_ids],
            dtype=np.float64,
        )

        # Compute pairwise distance matrix: (num_existing, num_new)
        dist_matrix = self._compute_distance_matrix(
            object_centroids, input_centroids
        )

        # Greedy assignment: sort by distance, assign closest pairs first
        rows = dist_matrix.min(axis=1).argsort()
        cols = dist_matrix.argmin(axis=1)[rows]

        used_rows: set = set()
        used_cols: set = set()

        for row, col in zip(rows, cols):
            if row in used_rows or col in used_cols:
                continue
            if dist_matrix[row, col] > self.max_distance:
                continue

            obj_id = object_ids[row]
            centroid, bbox, class_name, confidence = detections[col]
            old_centroid = self._objects[obj_id].centroid
            velocity = (
                centroid[0] - old_centroid[0],
                centroid[1] - old_centroid[1],
            )
            self._objects[obj_id].centroid = centroid
            self._objects[obj_id].bbox = bbox
            self._objects[obj_id].class_name = class_name
            self._objects[obj_id].confidence = confidence
            self._objects[obj_id].disappeared = 0
            self._objects[obj_id].velocity = velocity

            used_rows.add(row)
            used_cols.add(col)

        # Handle unmatched existing objects (not assigned to any detection)
        unused_rows = set(range(dist_matrix.shape[0])) - used_rows
        for row in unused_rows:
            obj_id = object_ids[row]
            self._objects[obj_id].disappeared += 1
            if self._objects[obj_id].disappeared > self.max_disappeared:
                self._deregister(obj_id)

        # Handle unmatched new detections (register as new objects)
        unused_cols = set(range(dist_matrix.shape[1])) - used_cols
        for col in unused_cols:
            centroid, bbox, class_name, confidence = detections[col]
            self._register(centroid, bbox, class_name, confidence)

        return dict(self._objects)

    def reset(self) -> None:
        """Clear all tracked objects and reset the ID counter."""
        self._objects.clear()
        self._next_object_id = 0

    @property
    def objects(self) -> Dict[int, TrackedObject]:
        """Read-only access to currently tracked objects."""
        return dict(self._objects)

    # -----------------------------------------------------------------
    # Internal helpers
    # -----------------------------------------------------------------

    def _register(
        self,
        centroid: Tuple[float, float],
        bbox: Tuple[float, float, float, float],
        class_name: str,
        confidence: float,
    ) -> int:
        """Register a brand-new object and return its ID."""
        obj = TrackedObject(
            object_id=self._next_object_id,
            centroid=centroid,
            bbox=bbox,
            class_name=class_name,
            confidence=confidence,
        )
        self._objects[self._next_object_id] = obj
        self._next_object_id += 1
        return obj.object_id

    def _deregister(self, object_id: int) -> None:
        """Remove a tracked object by its ID."""
        del self._objects[object_id]

    @staticmethod
    def _compute_distance_matrix(
        a: np.ndarray, b: np.ndarray
    ) -> np.ndarray:
        """
        Compute Euclidean distance matrix between two sets of 2D points.

        Args:
            a: (M, 2) array of existing centroids.
            b: (N, 2) array of new centroids.

        Returns:
            (M, N) distance matrix.
        """
        # Efficient vectorized computation
        diff = a[:, np.newaxis, :] - b[np.newaxis, :, :]
        return np.sqrt((diff ** 2).sum(axis=2))


# =============================================================================
# TrackerNode — ROS 2 Node
# =============================================================================

class TrackerNode(Node):
    """
    ROS 2 node that bridges YOLO detections with object tracking.

    Subscribes to Detection2DArray from yolo_node, runs the selected tracker
    (CentroidTracker or ByteTrackTracker), and publishes tracking commands
    as JSON-encoded String messages.

    The tracking command contains error offsets (normalized -1.0 to 1.0)
    representing how far the tracked target is from the image center.
    Downstream actuator nodes use these offsets to steer motors/servos.

    Published JSON schema for /tracking/command:
        {
            "target_id": int,          # Tracker-assigned object ID
            "error_x": float,          # Horizontal offset from center [-1, 1]
            "error_y": float,          # Vertical offset from center [-1, 1]
            "velocity_x": float,       # Horizontal velocity (px/frame)
            "velocity_y": float,       # Vertical velocity (px/frame)
            "bbox_width": float,       # Bounding box width (pixels)
            "bbox_height": float,      # Bounding box height (pixels)
            "class_name": str,         # YOLO class label
            "confidence": float,       # Detection confidence
            "timestamp": float         # ROS timestamp (seconds)
        }
    """

    def __init__(self, **kwargs) -> None:
        """Initialize the tracker node with parameters and ROS interfaces."""
        super().__init__('tracker_node', **kwargs)

        # -----------------------------------------------------------------
        # Declare Parameters
        # -----------------------------------------------------------------
        self.declare_parameter('tracking_target_class', 'person')
        self.declare_parameter('tracker_type', 'centroid')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)

        # CentroidTracker parameters
        self.declare_parameter('max_disappeared', 30)
        self.declare_parameter('max_distance', 80.0)

        # ByteTrackTracker parameters
        self.declare_parameter('track_high_thresh', 0.5)
        self.declare_parameter('track_low_thresh', 0.1)
        self.declare_parameter('match_thresh', 0.8)

        self._target_class: str = (
            self.get_parameter('tracking_target_class')
            .get_parameter_value().string_value
        )
        tracker_type: str = (
            self.get_parameter('tracker_type')
            .get_parameter_value().string_value
        )
        self._image_width: int = (
            self.get_parameter('image_width')
            .get_parameter_value().integer_value
        )
        self._image_height: int = (
            self.get_parameter('image_height')
            .get_parameter_value().integer_value
        )

        self.get_logger().info(f'Tracking target class: {self._target_class}')
        self.get_logger().info(
            f'Image size: {self._image_width}x{self._image_height}'
        )

        # -----------------------------------------------------------------
        # Initialize Tracker (Strategy Pattern)
        # -----------------------------------------------------------------
        if tracker_type == 'bytetrack':
            from yolo_rpi_core.bytetrack_tracker import ByteTrackTracker

            track_high_thresh = (
                self.get_parameter('track_high_thresh')
                .get_parameter_value().double_value
            )
            track_low_thresh = (
                self.get_parameter('track_low_thresh')
                .get_parameter_value().double_value
            )
            match_thresh = (
                self.get_parameter('match_thresh')
                .get_parameter_value().double_value
            )
            max_lost = (
                self.get_parameter('max_disappeared')
                .get_parameter_value().integer_value
            )

            self._tracker = ByteTrackTracker(
                track_high_thresh=track_high_thresh,
                track_low_thresh=track_low_thresh,
                match_thresh=match_thresh,
                max_lost=max_lost,
            )
            self.get_logger().info(
                f'Tracker: ByteTrack (high={track_high_thresh}, '
                f'low={track_low_thresh}, match={match_thresh})'
            )
        else:
            max_disappeared = (
                self.get_parameter('max_disappeared')
                .get_parameter_value().integer_value
            )
            max_distance = (
                self.get_parameter('max_distance')
                .get_parameter_value().double_value
            )

            self._tracker = CentroidTracker(
                max_disappeared=max_disappeared,
                max_distance=max_distance,
            )
            self.get_logger().info(
                f'Tracker: Centroid (max_dist={max_distance}, '
                f'max_disappeared={max_disappeared})'
            )

        # -----------------------------------------------------------------
        # CV Bridge (for debug image rendering)
        # -----------------------------------------------------------------
        self._bridge = CvBridge()
        self._latest_frame: Optional[np.ndarray] = None

        # -----------------------------------------------------------------
        # QoS Profiles
        # -----------------------------------------------------------------
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # -----------------------------------------------------------------
        # Subscribers
        # -----------------------------------------------------------------
        self._detection_sub = self.create_subscription(
            Detection2DArray,
            '/yolo/detections',
            self._detection_callback,
            10,
        )

        self._image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self._image_callback,
            sensor_qos,
        )

        # -----------------------------------------------------------------
        # Publishers
        # -----------------------------------------------------------------
        self._command_pub = self.create_publisher(
            String,
            '/tracking/command',
            10,
        )

        self._debug_image_pub = self.create_publisher(
            Image,
            '/tracking/debug_image',
            10,
        )

        self.get_logger().info('TrackerNode initialized and ready!')

    # -----------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------

    def _image_callback(self, msg: Image) -> None:
        """Cache the latest camera frame for debug image rendering."""
        try:
            self._latest_frame = self._bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')

    def _detection_callback(self, msg: Detection2DArray) -> None:
        """
        Process incoming YOLO detections and update the tracker.

        Filters detections by target class, updates the centroid tracker,
        selects the primary target (closest to image center), and publishes
        a tracking command.
        """
        # Parse detections into tracker-compatible format
        detections = self._parse_detections(msg)

        # Update tracker with new detections
        tracked_objects = self._tracker.update(detections)

        # Select primary tracking target (closest to center)
        target = self._select_primary_target(tracked_objects)

        if target is not None:
            # Compute error offsets and publish command
            command = self._build_tracking_command(target, msg.header)
            self._command_pub.publish(command)

        # Publish debug image with tracking overlay
        self._publish_debug_image(tracked_objects, msg.header)

    # -----------------------------------------------------------------
    # Detection Parsing
    # -----------------------------------------------------------------

    def _parse_detections(
        self, msg: Detection2DArray
    ) -> List[Tuple[Tuple[float, float], Tuple[float, float, float, float], str, float]]:
        """
        Convert Detection2DArray into a list of (centroid, bbox, class, conf).

        Filters detections to only include the configured target class.
        If target class is 'all', includes all detections.
        """
        detections = []

        for det in msg.detections:
            if not det.results:
                continue

            # Take the top hypothesis
            hyp = det.results[0]
            class_name = hyp.hypothesis.class_id
            confidence = hyp.hypothesis.score

            # Filter by target class (skip if not the class we want)
            if (self._target_class != 'all'
                    and class_name != self._target_class):
                continue

            # Extract centroid and bbox from Detection2D
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            w = det.bbox.size_x
            h = det.bbox.size_y

            x1 = cx - w / 2.0
            y1 = cy - h / 2.0
            x2 = cx + w / 2.0
            y2 = cy + h / 2.0

            detections.append(
                ((cx, cy), (x1, y1, x2, y2), class_name, confidence)
            )

        return detections

    # -----------------------------------------------------------------
    # Target Selection
    # -----------------------------------------------------------------

    def _select_primary_target(
        self, tracked_objects: Dict[int, TrackedObject]
    ) -> Optional[TrackedObject]:
        """
        Select the primary tracking target among all tracked objects.

        Strategy: Pick the object closest to the image center.
        This ensures the tracker follows the most "centered" subject,
        which is usually the intended target in a following robot.

        Returns:
            The TrackedObject closest to center, or None if no objects.
        """
        if not tracked_objects:
            return None

        center_x = self._image_width / 2.0
        center_y = self._image_height / 2.0

        best_target: Optional[TrackedObject] = None
        best_distance = float('inf')

        for obj in tracked_objects.values():
            # Skip objects that have temporarily disappeared
            if obj.disappeared > 0:
                continue

            dx = obj.centroid[0] - center_x
            dy = obj.centroid[1] - center_y
            dist = math.sqrt(dx * dx + dy * dy)

            if dist < best_distance:
                best_distance = dist
                best_target = obj

        return best_target

    # -----------------------------------------------------------------
    # Command Building
    # -----------------------------------------------------------------

    def _build_tracking_command(
        self, target: TrackedObject, header
    ) -> String:
        """
        Build a JSON tracking command message.

        The error values are normalized to [-1.0, 1.0]:
            - error_x < 0 → target is LEFT of center
            - error_x > 0 → target is RIGHT of center
            - error_y < 0 → target is ABOVE center
            - error_y > 0 → target is BELOW center

        Args:
            target: The selected TrackedObject.
            header: ROS message header for timestamp.

        Returns:
            std_msgs/String containing JSON payload.
        """
        center_x = self._image_width / 2.0
        center_y = self._image_height / 2.0

        # Normalize error to [-1.0, 1.0]
        error_x = (target.centroid[0] - center_x) / center_x
        error_y = (target.centroid[1] - center_y) / center_y

        # Clamp to valid range
        error_x = max(-1.0, min(1.0, error_x))
        error_y = max(-1.0, min(1.0, error_y))

        # Compute bbox dimensions
        bbox_width = target.bbox[2] - target.bbox[0]
        bbox_height = target.bbox[3] - target.bbox[1]

        # Timestamp from header
        timestamp = header.stamp.sec + header.stamp.nanosec * 1e-9

        payload = {
            'target_id': target.object_id,
            'error_x': round(error_x, 4),
            'error_y': round(error_y, 4),
            'velocity_x': round(target.velocity[0], 2),
            'velocity_y': round(target.velocity[1], 2),
            'bbox_width': round(bbox_width, 1),
            'bbox_height': round(bbox_height, 1),
            'class_name': target.class_name,
            'confidence': round(target.confidence, 3),
            'timestamp': round(timestamp, 6),
        }

        msg = String()
        msg.data = json.dumps(payload)
        return msg

    # -----------------------------------------------------------------
    # Debug Image
    # -----------------------------------------------------------------

    def _publish_debug_image(
        self, tracked_objects: Dict[int, TrackedObject], header
    ) -> None:
        """
        Render tracking overlay on the latest camera frame and publish.

        Draws:
        - Bounding box with tracking ID
        - Centroid dot
        - Velocity arrow
        - Class name + confidence
        """
        if self._latest_frame is None:
            return

        try:
            import cv2
        except ImportError:
            return

        frame = self._latest_frame.copy()

        for obj in tracked_objects.values():
            if obj.disappeared > 0:
                continue

            x1, y1, x2, y2 = [int(v) for v in obj.bbox]
            cx, cy = int(obj.centroid[0]), int(obj.centroid[1])

            # Draw bounding box (green)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Draw centroid
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

            # Draw velocity arrow
            vx, vy = obj.velocity
            if abs(vx) > 1.0 or abs(vy) > 1.0:
                arrow_scale = 3.0
                end_x = int(cx + vx * arrow_scale)
                end_y = int(cy + vy * arrow_scale)
                cv2.arrowedLine(
                    frame, (cx, cy), (end_x, end_y),
                    (255, 255, 0), 2, tipLength=0.3,
                )

            # Label: ID + class + confidence
            label = (
                f'ID:{obj.object_id} {obj.class_name} '
                f'{obj.confidence:.2f}'
            )
            cv2.putText(
                frame, label, (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2,
            )

        # Draw image center crosshair
        cx_img = self._image_width // 2
        cy_img = self._image_height // 2
        cv2.drawMarker(
            frame, (cx_img, cy_img), (0, 0, 255),
            cv2.MARKER_CROSS, 20, 2,
        )

        try:
            debug_msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            debug_msg.header = header
            self._debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'Failed to publish debug image: {e}')


# =============================================================================
# Entry Point
# =============================================================================

def main(args: Optional[List[str]] = None) -> None:
    """Entry point for the tracker node."""
    rclpy.init(args=args)

    try:
        node = TrackerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'TrackerNode error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
