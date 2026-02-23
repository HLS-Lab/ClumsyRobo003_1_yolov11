#!/usr/bin/env python3
"""
ByteTrack-style Multi-Object Tracker (CPU-only, numpy-only).

Implements the core ByteTrack algorithm:
    1. Kalman Filter for motion prediction (constant-velocity model).
    2. IoU (Intersection-over-Union) for detection-to-track matching.
    3. Two-stage association:
       - Stage 1: Match high-confidence detections to predicted tracks.
       - Stage 2: Match low-confidence detections to remaining tracks.

This implementation requires ONLY numpy — no scipy, no deep learning.
Designed for Raspberry Pi 4 (CPU inference, NO CUDA).

Reference:
    ByteTrack: Multi-Object Tracking by Associating Every Detection Box
    (Zhang et al., ECCV 2022)

Compatible with: ROS 2 Jazzy, Python 3.12
Target Platform: Raspberry Pi 4 (CPU only)
"""

from __future__ import annotations

from enum import Enum, auto
from typing import Dict, List, Optional, Tuple

import numpy as np

# Import shared TrackedObject dataclass from tracker_node
from yolo_rpi_core.tracker_node import TrackedObject


# =============================================================================
# Kalman Filter — numpy-only, constant-velocity model
# =============================================================================

class KalmanFilter:
    """
    Lightweight Kalman Filter for bounding box tracking.

    State vector (8D):
        [cx, cy, aspect_ratio, height, vx, vy, va, vh]

        - (cx, cy):        Bounding box center
        - aspect_ratio:    Width / height ratio
        - height:          Bounding box height
        - (vx, vy, va, vh): Velocities of the above

    Measurement vector (4D):
        [cx, cy, aspect_ratio, height]

    Uses a constant-velocity motion model with no control input.
    """

    # Noise scaling factors (tuned for typical video tracking)
    _STD_WEIGHT_POSITION = 1.0 / 20.0
    _STD_WEIGHT_VELOCITY = 1.0 / 160.0

    def __init__(self) -> None:
        """Initialize the Kalman Filter matrices."""
        ndim = 4  # measurement dimensions
        dt = 1.0  # time step (1 frame)

        # State transition matrix F: 8×8
        # Maps state at t to state at t+1 (constant velocity model)
        self._F = np.eye(2 * ndim, dtype=np.float64)
        for i in range(ndim):
            self._F[i, ndim + i] = dt

        # Measurement matrix H: 4×8
        # Extracts [cx, cy, a, h] from the full state
        self._H = np.eye(ndim, 2 * ndim, dtype=np.float64)

    def initiate(
        self, measurement: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Create a new track from an unassociated measurement.

        Args:
            measurement: (4,) array [cx, cy, aspect_ratio, height].

        Returns:
            (mean, covariance): Initial state estimate and uncertainty.
        """
        mean_pos = measurement.copy()
        mean_vel = np.zeros_like(mean_pos)
        mean = np.concatenate([mean_pos, mean_vel])

        # Initial covariance: large uncertainty for velocities
        std = np.array([
            2.0 * self._STD_WEIGHT_POSITION * measurement[3],
            2.0 * self._STD_WEIGHT_POSITION * measurement[3],
            1e-2,
            2.0 * self._STD_WEIGHT_POSITION * measurement[3],
            10.0 * self._STD_WEIGHT_VELOCITY * measurement[3],
            10.0 * self._STD_WEIGHT_VELOCITY * measurement[3],
            1e-5,
            10.0 * self._STD_WEIGHT_VELOCITY * measurement[3],
        ], dtype=np.float64)

        covariance = np.diag(std ** 2)
        return mean, covariance

    def predict(
        self, mean: np.ndarray, covariance: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Predict the next state using the motion model.

        Args:
            mean: (8,) current state estimate.
            covariance: (8,8) current state covariance.

        Returns:
            (predicted_mean, predicted_covariance).
        """
        # Process noise
        std = np.array([
            self._STD_WEIGHT_POSITION * mean[3],
            self._STD_WEIGHT_POSITION * mean[3],
            1e-2,
            self._STD_WEIGHT_POSITION * mean[3],
            self._STD_WEIGHT_VELOCITY * mean[3],
            self._STD_WEIGHT_VELOCITY * mean[3],
            1e-5,
            self._STD_WEIGHT_VELOCITY * mean[3],
        ], dtype=np.float64)
        Q = np.diag(std ** 2)

        mean = self._F @ mean
        covariance = self._F @ covariance @ self._F.T + Q
        return mean, covariance

    def update(
        self,
        mean: np.ndarray,
        covariance: np.ndarray,
        measurement: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Correct the predicted state with a matched measurement.

        Args:
            mean: (8,) predicted state.
            covariance: (8,8) predicted covariance.
            measurement: (4,) observed [cx, cy, aspect_ratio, height].

        Returns:
            (corrected_mean, corrected_covariance).
        """
        # Measurement noise
        std = np.array([
            self._STD_WEIGHT_POSITION * mean[3],
            self._STD_WEIGHT_POSITION * mean[3],
            1e-1,
            self._STD_WEIGHT_POSITION * mean[3],
        ], dtype=np.float64)
        R = np.diag(std ** 2)

        # Innovation (measurement residual)
        projected_mean = self._H @ mean
        innovation = measurement - projected_mean

        # Innovation covariance S = H @ P @ H^T + R
        S = self._H @ covariance @ self._H.T + R

        # Kalman gain K = P @ H^T @ S^{-1}
        K = covariance @ self._H.T @ np.linalg.inv(S)

        # Corrected state
        new_mean = mean + K @ innovation
        new_covariance = covariance - K @ self._H @ covariance

        return new_mean, new_covariance


# =============================================================================
# Track Lifecycle States
# =============================================================================

class TrackState(Enum):
    """Lifecycle states for a single tracked object (STrack)."""
    NEW = auto()       # Just created, not yet confirmed
    TRACKED = auto()   # Actively matched across frames
    LOST = auto()      # Temporarily unmatched (may recover)
    REMOVED = auto()   # Permanently removed


# =============================================================================
# STrack — Single Track Object with Kalman State
# =============================================================================

class STrack:
    """
    A single tracked object with Kalman filter state.

    Manages the lifecycle (NEW → TRACKED → LOST → REMOVED) and stores
    the Kalman filter mean/covariance for motion prediction.

    Attributes:
        track_id:    Unique ID assigned by ByteTrackTracker.
        state:       Current lifecycle state.
        class_name:  YOLO class label.
        confidence:  Latest detection confidence.
        frame_id:    Frame number of the last update.
    """

    _shared_kalman = KalmanFilter()

    def __init__(
        self,
        bbox: Tuple[float, float, float, float],
        confidence: float,
        class_name: str,
    ) -> None:
        """
        Initialize a new STrack.

        Args:
            bbox: (x1, y1, x2, y2) bounding box.
            confidence: Detection confidence score.
            class_name: YOLO class label.
        """
        self.track_id: int = 0  # Assigned later by the tracker
        self.state: TrackState = TrackState.NEW

        self.class_name: str = class_name
        self.confidence: float = confidence

        self._bbox: Tuple[float, float, float, float] = bbox

        # Kalman state
        measurement = self._bbox_to_measurement(bbox)
        self._mean: np.ndarray
        self._covariance: np.ndarray
        self._mean, self._covariance = self._shared_kalman.initiate(measurement)

        # Lifecycle counters
        self.frame_id: int = 0
        self.tracklet_len: int = 0
        self._start_frame: int = 0

    # -----------------------------------------------------------------
    # Geometry helpers
    # -----------------------------------------------------------------

    @staticmethod
    def _bbox_to_measurement(
        bbox: Tuple[float, float, float, float]
    ) -> np.ndarray:
        """Convert (x1, y1, x2, y2) to [cx, cy, aspect_ratio, height]."""
        x1, y1, x2, y2 = bbox
        w = x2 - x1
        h = y2 - y1
        cx = x1 + w / 2.0
        cy = y1 + h / 2.0
        aspect = w / max(h, 1e-6)
        return np.array([cx, cy, aspect, h], dtype=np.float64)

    @staticmethod
    def _measurement_to_bbox(measurement: np.ndarray) -> Tuple[float, float, float, float]:
        """Convert [cx, cy, aspect_ratio, height] to (x1, y1, x2, y2)."""
        cx, cy, aspect, h = measurement[:4]
        w = aspect * h
        x1 = cx - w / 2.0
        y1 = cy - h / 2.0
        x2 = cx + w / 2.0
        y2 = cy + h / 2.0
        return (x1, y1, x2, y2)

    @property
    def bbox(self) -> Tuple[float, float, float, float]:
        """Current bounding box (x1, y1, x2, y2) from Kalman state."""
        return self._measurement_to_bbox(self._mean[:4])

    @property
    def centroid(self) -> Tuple[float, float]:
        """Current center (cx, cy) from Kalman state."""
        return (float(self._mean[0]), float(self._mean[1]))

    @property
    def velocity(self) -> Tuple[float, float]:
        """Current velocity (vx, vy) from Kalman state."""
        return (float(self._mean[4]), float(self._mean[5]))

    # -----------------------------------------------------------------
    # Kalman operations
    # -----------------------------------------------------------------

    def predict(self) -> None:
        """Advance the Kalman state by one time step."""
        self._mean, self._covariance = self._shared_kalman.predict(
            self._mean, self._covariance
        )

    def update(
        self,
        bbox: Tuple[float, float, float, float],
        confidence: float,
        class_name: str,
        frame_id: int,
    ) -> None:
        """
        Correct Kalman state with a matched detection.

        Args:
            bbox: Matched detection bounding box (x1, y1, x2, y2).
            confidence: Detection confidence.
            class_name: YOLO class label.
            frame_id: Current frame number.
        """
        measurement = self._bbox_to_measurement(bbox)
        self._mean, self._covariance = self._shared_kalman.update(
            self._mean, self._covariance, measurement
        )
        self._bbox = bbox
        self.confidence = confidence
        self.class_name = class_name
        self.frame_id = frame_id
        self.tracklet_len += 1
        self.state = TrackState.TRACKED

    def activate(self, track_id: int, frame_id: int) -> None:
        """Activate a new track with an assigned ID."""
        self.track_id = track_id
        self.frame_id = frame_id
        self._start_frame = frame_id
        self.tracklet_len = 0
        self.state = TrackState.TRACKED

    def mark_lost(self) -> None:
        """Mark the track as lost (temporarily unmatched)."""
        self.state = TrackState.LOST

    def mark_removed(self) -> None:
        """Mark the track as permanently removed."""
        self.state = TrackState.REMOVED


# =============================================================================
# IoU Computation — vectorized numpy
# =============================================================================

def _compute_iou_matrix(
    bboxes_a: np.ndarray,
    bboxes_b: np.ndarray,
) -> np.ndarray:
    """
    Compute IoU (Intersection over Union) between two sets of bboxes.

    Args:
        bboxes_a: (M, 4) array of [x1, y1, x2, y2].
        bboxes_b: (N, 4) array of [x1, y1, x2, y2].

    Returns:
        (M, N) IoU matrix.
    """
    if len(bboxes_a) == 0 or len(bboxes_b) == 0:
        return np.zeros((len(bboxes_a), len(bboxes_b)), dtype=np.float64)

    # Broadcast intersection
    x1 = np.maximum(bboxes_a[:, 0:1], bboxes_b[:, 0].T)  # (M, N)
    y1 = np.maximum(bboxes_a[:, 1:2], bboxes_b[:, 1].T)
    x2 = np.minimum(bboxes_a[:, 2:3], bboxes_b[:, 2].T)
    y2 = np.minimum(bboxes_a[:, 3:4], bboxes_b[:, 3].T)

    inter = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)

    area_a = (bboxes_a[:, 2] - bboxes_a[:, 0]) * (bboxes_a[:, 3] - bboxes_a[:, 1])
    area_b = (bboxes_b[:, 2] - bboxes_b[:, 0]) * (bboxes_b[:, 3] - bboxes_b[:, 1])

    union = area_a[:, np.newaxis] + area_b[np.newaxis, :] - inter
    iou = inter / np.maximum(union, 1e-6)

    return iou


# =============================================================================
# Linear Assignment — greedy IoU matching (no scipy required)
# =============================================================================

def _greedy_assignment(
    cost_matrix: np.ndarray,
    thresh: float,
) -> Tuple[List[Tuple[int, int]], List[int], List[int]]:
    """
    Greedy assignment on a cost matrix (IoU-based, higher = better).

    Unlike Hungarian algorithm, this is a simple greedy approach that
    assigns the best match first, then removes both row and column.

    Args:
        cost_matrix: (M, N) IoU matrix (higher = better match).
        thresh: Minimum IoU threshold to accept a match.

    Returns:
        (matches, unmatched_rows, unmatched_cols):
            - matches: List of (row, col) index pairs.
            - unmatched_rows: Row indices with no match.
            - unmatched_cols: Column indices with no match.
    """
    if cost_matrix.size == 0:
        return (
            [],
            list(range(cost_matrix.shape[0])),
            list(range(cost_matrix.shape[1])),
        )

    matches: List[Tuple[int, int]] = []
    used_rows: set = set()
    used_cols: set = set()

    # Flatten and sort by descending IoU (best matches first)
    num_rows, num_cols = cost_matrix.shape
    flat_indices = np.argsort(-cost_matrix.ravel())

    for flat_idx in flat_indices:
        row = int(flat_idx // num_cols)
        col = int(flat_idx % num_cols)

        if row in used_rows or col in used_cols:
            continue
        if cost_matrix[row, col] < thresh:
            break  # All remaining are below threshold

        matches.append((row, col))
        used_rows.add(row)
        used_cols.add(col)

    unmatched_rows = [i for i in range(num_rows) if i not in used_rows]
    unmatched_cols = [j for j in range(num_cols) if j not in used_cols]

    return matches, unmatched_rows, unmatched_cols


# =============================================================================
# ByteTrackTracker — Main Tracker Class
# =============================================================================

class ByteTrackTracker:
    """
    ByteTrack-style multi-object tracker with two-stage association.

    Algorithm overview:
        1. Predict all active tracks forward using Kalman filter.
        2. Split new detections into HIGH and LOW confidence groups.
        3. Stage 1: Match HIGH-confidence detections to predicted tracks
           using IoU. Unmatched high-conf detections become new tracks.
        4. Stage 2: Match LOW-confidence detections to REMAINING
           unmatched tracks using IoU. This recovers partially occluded
           objects that produce low-confidence detections.
        5. Unmatched tracks increment their lost counter.
        6. Tracks lost for too many frames are removed.

    This is what makes ByteTrack special: it does NOT discard
    low-confidence detections like most trackers. Instead, it uses
    them in a second matching pass to maintain tracking through
    occlusion.

    Args:
        track_high_thresh: Confidence threshold for high-conf detections.
        track_low_thresh:  Confidence threshold for low-conf detections.
                           Detections below this are discarded entirely.
        match_thresh:      Minimum IoU for a valid match (0.0-1.0).
                           Lower = more lenient matching.
        max_lost:          Frames to keep lost tracks before removal.

    Public API is identical to CentroidTracker:
        update(detections) → Dict[int, TrackedObject]
    """

    def __init__(
        self,
        track_high_thresh: float = 0.5,
        track_low_thresh: float = 0.1,
        match_thresh: float = 0.8,
        max_lost: int = 30,
    ) -> None:
        self._track_high_thresh = track_high_thresh
        self._track_low_thresh = track_low_thresh
        self._match_thresh = match_thresh
        self._max_lost = max_lost

        self._tracked_stracks: List[STrack] = []
        self._lost_stracks: List[STrack] = []

        self._next_id: int = 0
        self._frame_id: int = 0

    # -----------------------------------------------------------------
    # Public API (same interface as CentroidTracker)
    # -----------------------------------------------------------------

    def update(
        self,
        detections: List[Tuple[
            Tuple[float, float],
            Tuple[float, float, float, float],
            str,
            float,
        ]],
    ) -> Dict[int, TrackedObject]:
        """
        Update tracker with new detections (same signature as CentroidTracker).

        Args:
            detections: List of (centroid, bbox, class_name, confidence).

        Returns:
            Dict mapping track_id → TrackedObject for all active tracks.
        """
        self._frame_id += 1

        # ----- Step 0: Split detections by confidence -----
        high_dets: List[Tuple[Tuple[float, float], Tuple[float, float, float, float], str, float]] = []
        low_dets: List[Tuple[Tuple[float, float], Tuple[float, float, float, float], str, float]] = []

        for det in detections:
            _, _, _, conf = det
            if conf >= self._track_high_thresh:
                high_dets.append(det)
            elif conf >= self._track_low_thresh:
                low_dets.append(det)
            # else: discard (below low threshold)

        # ----- Step 1: Predict all existing tracks -----
        all_stracks = self._tracked_stracks + self._lost_stracks
        for strack in all_stracks:
            strack.predict()

        # ----- Step 2: Stage 1 — match HIGH-conf detections to tracks -----
        matched_tracks_1, unmatched_track_indices_1, unmatched_det_indices_1 = (
            self._match_detections(all_stracks, high_dets)
        )

        # Update matched tracks
        for track_idx, det_idx in matched_tracks_1:
            strack = all_stracks[track_idx]
            _, bbox, class_name, conf = high_dets[det_idx]
            strack.update(bbox, conf, class_name, self._frame_id)

        # ----- Step 3: Stage 2 — match LOW-conf detections to REMAINING tracks -----
        remaining_stracks = [all_stracks[i] for i in unmatched_track_indices_1]

        matched_tracks_2, unmatched_track_indices_2, _ = (
            self._match_detections(remaining_stracks, low_dets)
        )

        for track_idx, det_idx in matched_tracks_2:
            strack = remaining_stracks[track_idx]
            _, bbox, class_name, conf = low_dets[det_idx]
            strack.update(bbox, conf, class_name, self._frame_id)

        # ----- Step 4: Handle unmatched tracks -----
        still_unmatched = [remaining_stracks[i] for i in unmatched_track_indices_2]
        for strack in still_unmatched:
            if strack.state == TrackState.TRACKED:
                strack.mark_lost()

        # ----- Step 5: Create new tracks from unmatched HIGH-conf detections -----
        for det_idx in unmatched_det_indices_1:
            _, bbox, class_name, conf = high_dets[det_idx]
            new_strack = STrack(bbox, conf, class_name)
            new_strack.activate(self._next_id, self._frame_id)
            self._next_id += 1
            self._tracked_stracks.append(new_strack)

        # ----- Step 6: Update track lists -----
        self._tracked_stracks = [
            s for s in all_stracks if s.state == TrackState.TRACKED
        ] + [
            s for s in self._tracked_stracks
            if s.state == TrackState.TRACKED and s not in all_stracks
        ]

        self._lost_stracks = [
            s for s in all_stracks if s.state == TrackState.LOST
        ]

        # Remove tracks lost for too long
        self._lost_stracks = [
            s for s in self._lost_stracks
            if (self._frame_id - s.frame_id) <= self._max_lost
        ]

        # ----- Build output (same format as CentroidTracker) -----
        return self._build_output()

    def reset(self) -> None:
        """Clear all tracked objects and reset the ID counter."""
        self._tracked_stracks.clear()
        self._lost_stracks.clear()
        self._next_id = 0
        self._frame_id = 0

    @property
    def objects(self) -> Dict[int, TrackedObject]:
        """Read-only access to currently tracked objects."""
        return self._build_output()

    # -----------------------------------------------------------------
    # Internal helpers
    # -----------------------------------------------------------------

    def _match_detections(
        self,
        stracks: List[STrack],
        dets: List[Tuple[Tuple[float, float], Tuple[float, float, float, float], str, float]],
    ) -> Tuple[List[Tuple[int, int]], List[int], List[int]]:
        """
        Match detections to tracks using IoU.

        Args:
            stracks: List of existing STrack objects.
            dets: List of detection tuples.

        Returns:
            (matches, unmatched_track_indices, unmatched_det_indices).
        """
        if len(stracks) == 0 or len(dets) == 0:
            return [], list(range(len(stracks))), list(range(len(dets)))

        # Build bbox arrays
        track_bboxes = np.array(
            [s.bbox for s in stracks], dtype=np.float64
        )
        det_bboxes = np.array(
            [d[1] for d in dets], dtype=np.float64
        )

        # Compute IoU matrix
        iou_matrix = _compute_iou_matrix(track_bboxes, det_bboxes)

        # Greedy match (IoU >= 1 - match_thresh)
        # Note: match_thresh is the IoU threshold below which we reject
        return _greedy_assignment(
            iou_matrix, thresh=(1.0 - self._match_thresh)
        )

    def _build_output(self) -> Dict[int, TrackedObject]:
        """
        Convert internal STrack list to TrackedObject dict.

        Returns:
            Dict mapping track_id → TrackedObject (same as CentroidTracker).
        """
        result: Dict[int, TrackedObject] = {}

        for strack in self._tracked_stracks:
            if strack.state != TrackState.TRACKED:
                continue

            bbox = strack.bbox
            obj = TrackedObject(
                object_id=strack.track_id,
                centroid=strack.centroid,
                bbox=bbox,
                class_name=strack.class_name,
                confidence=strack.confidence,
                disappeared=0,
                velocity=strack.velocity,
            )
            result[strack.track_id] = obj

        # Include lost tracks with disappeared > 0
        for strack in self._lost_stracks:
            lost_frames = self._frame_id - strack.frame_id
            bbox = strack.bbox
            obj = TrackedObject(
                object_id=strack.track_id,
                centroid=strack.centroid,
                bbox=bbox,
                class_name=strack.class_name,
                confidence=strack.confidence,
                disappeared=lost_frames,
                velocity=strack.velocity,
            )
            result[strack.track_id] = obj

        return result
