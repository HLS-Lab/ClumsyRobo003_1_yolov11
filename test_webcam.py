#!/usr/bin/env python3
"""
Standalone webcam test script for YOLOv11 + tracking pipeline.

This uses the host PC's webcam (typically cv2.VideoCapture(0)) to run inference
using the same YOLO model and tracker classes as the ROS nodes, but without
requiring a ROS 2 environment.

Requirements:
    - uv (Python project manager)
    - ultralytics (implied by model requirement)
    - opencv-python
"""

import sys
import time
from typing import Dict, List, Tuple

import cv2
import numpy as np

# Suppress ultralytics welcome messages
import os
os.environ['YOLO_VERBOSE'] = 'False'

try:
    from ultralytics import YOLO
except ImportError as e:
    print("Error: ultralytics is not installed.")
    print("Please ensure your environment is set up (e.g., via uv run python test_webcam.py).")
    print(f"Details: {e}")
    sys.exit(1)

from pathlib import Path

# Add the yolo_rpi_core directory to the Python path so we can import the module directly
sys.path.insert(0, str(Path(__file__).resolve().parent / 'yolo_rpi_core'))

# Import tracking utilities from our core module
try:
    from yolo_rpi_core.tracked_object import TrackedObject
    from yolo_rpi_core.bytetrack_tracker import ByteTrackTracker
except ImportError as e:
    print("Error: Could not import yolo_rpi_core modules.")
    print("Ensure this script is run from the project root and the module is in PYTHONPATH.")
    print(f"Details: {e}")
    sys.exit(1)


def main():
    # -------------------------------------------------------------------------
    # Configuration
    # -------------------------------------------------------------------------
    model_path = 'yolo11n.pt'
    target_class = 'person'  # Change to 'all' to track everything
    conf_threshold = 0.5
    
    # Tracker configuration (matching tracker_node.py defaults for ByteTrack)
    track_high_thresh = 0.5
    track_low_thresh = 0.1
    match_thresh = 0.8
    max_lost = 30

    print(f"Loading YOLO model: {model_path} (Device: CPU)")
    try:
        model = YOLO(model_path)
        model.to('cpu')
    except Exception as e:
        print(f"Failed to load YOLO model: {e}")
        return

    # Initialize tracker
    print(f"Initializing ByteTrack Tracker (high={track_high_thresh}, low={track_low_thresh}, match={match_thresh})")
    tracker = ByteTrackTracker(
        track_high_thresh=track_high_thresh,
        track_low_thresh=track_low_thresh,
        match_thresh=match_thresh,
        max_lost=max_lost,
    )

    # -------------------------------------------------------------------------
    # Camera Initialization
    # -------------------------------------------------------------------------
    # Iterate through potential external camera indices first, 
    # then fallback to the built-in webcam (0).
    cap = None
    for camera_id in [1, 2, 3, 4, 0]:
        print(f"Trying camera index {camera_id}...")
        temp_cap = cv2.VideoCapture(camera_id)
        if temp_cap.isOpened():
            ret, _ = temp_cap.read()
            if ret:
                cap = temp_cap
                print(f"Successfully connected to camera index {camera_id}")
                break
            else:
                temp_cap.release()
        else:
            temp_cap.release()

    if cap is None:
        print("Error: Could not open any webcam.")
        return
    
    # Try to set higher resolution (e.g., 640x480 or 1280x720)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f"Webcam initialized at {actual_width}x{actual_height}")
    print("Press 'q' to quit.")

    # Frame timing
    prev_time = time.time()
    
    # -------------------------------------------------------------------------
    # Inference Loop
    # -------------------------------------------------------------------------
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Failed to grab frame.")
                break

            # Calculate FPS
            curr_time = time.time()
            fps = 1.0 / (curr_time - prev_time)
            prev_time = curr_time

            # 1. Run YOLO detection
            results = model(frame, verbose=False, conf=conf_threshold)

            # 2. Parse detections for tracker
            tracker_inputs: List[Tuple[Tuple[float, float], Tuple[float, float, float, float], str, float]] = []
            
            if results and len(results) > 0:
                result = results[0]
                class_names = model.names
                
                if result.boxes is not None:
                    for box in result.boxes:
                        # Extract data
                        xyxy = box.xyxy[0].cpu().numpy()
                        x1, y1, x2, y2 = xyxy
                        conf = float(box.conf[0].item())
                        class_id = int(box.cls[0].item())
                        class_name = class_names.get(class_id, f'class_{class_id}')
                        
                        # Filter by class
                        if target_class != 'all' and class_name != target_class:
                            continue
                            
                        # Calculate centroid
                        cx = (x1 + x2) / 2.0
                        cy = (y1 + y2) / 2.0
                        
                        tracker_inputs.append(
                            ((cx, cy), (float(x1), float(y1), float(x2), float(y2)), class_name, float(conf))
                        )

            # 3. Update Tracker
            tracked_objects = tracker.update(tracker_inputs)

            # 4. Visualization
            draw_frame = frame.copy()
            
            # Draw tracked objects
            for obj in tracked_objects.values():
                # Skip disappeared objects (optional, or render them with dashed lines/lower opacity)
                if obj.disappeared > 0:
                    continue
                    
                x1, y1, x2, y2 = [int(v) for v in obj.bbox]
                cx, cy = int(obj.centroid[0]), int(obj.centroid[1])
                vx, vy = obj.velocity

                # Bounding box (Green)
                cv2.rectangle(draw_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # Centroid (Red dot)
                cv2.circle(draw_frame, (cx, cy), 5, (0, 0, 255), -1)

                # Velocity arrow (Cyan)
                if abs(vx) > 1.0 or abs(vy) > 1.0:
                    arrow_scale = 3.0
                    end_x = int(cx + vx * arrow_scale)
                    end_y = int(cy + vy * arrow_scale)
                    cv2.arrowedLine(
                        draw_frame, (cx, cy), (end_x, end_y),
                        (255, 255, 0), 2, tipLength=0.3
                    )

                # Label background
                label = f"ID:{obj.object_id} {obj.class_name} {obj.confidence:.2f}"
                (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(draw_frame, (x1, y1 - 20), (x1 + w, y1), (0, 255, 0), -1)
                
                # Text (Black)
                cv2.putText(
                    draw_frame, label, (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1
                )

            # Draw center crosshair
            cx_img = actual_width // 2
            cy_img = actual_height // 2
            cv2.drawMarker(
                draw_frame, (cx_img, cy_img), (0, 0, 255),
                cv2.MARKER_CROSS, 20, 2
            )

            # Draw FPS
            cv2.putText(
                draw_frame, f"FPS: {fps:.1f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2
            )
            cv2.putText(
                draw_frame, f"Tracking: {len([o for o in tracked_objects.values() if o.disappeared == 0])}", 
                (10, 65),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2
            )

            # Display
            cv2.imshow('YOLO + ByteTrack', draw_frame)

            # Process events, look for 'q' to quit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        print("Releasing resources...")
        cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
