from dataclasses import dataclass
from typing import Tuple

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
