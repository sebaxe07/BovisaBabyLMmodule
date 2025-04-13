import cv2
import numpy as np
import time
import argparse
import sys
import os

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from ultralytics import YOLO
from utils.colored_logger import log_debug, log_info, log_error

def calibrate_distance():
    """Standalone tool to calibrate the focal length for distance estimation"""
    log_info("CALIBRATION", "Starting distance calibration tool")
    
    # Parse arguments
    parser = argparse.ArgumentParser(description="Calibrate camera for distance estimation")
    parser.add_argument("--model", default="yolov8n.onnx", help="Path to YOLOv8 model")
    parser.add_argument("--confidence", type=float, default=0.5, help="Detection confidence threshold")
    parser.add_argument("--camera", type=int, default=0, help="Camera device ID")
    args = parser.parse_args()
    
    # Initialize camera
    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        log_error("CALIBRATION", "Failed to open camera")
        return None
    
    # Get camera resolution
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    log_info("CALIBRATION", f"Camera resolution: {frame_width}x{frame_height}")
    
    # Load YOLO model
    try:
        model = YOLO(args.model, task='detect')
        log_info("CALIBRATION", f"Loaded model: {args.model}")
    except Exception as e:
        log_error("CALIBRATION", f"Failed to load model: {e}")
        return None
    
    # Ask for calibration parameters
    print("\n===== DISTANCE CALIBRATION =====")
    print("This tool will calibrate the camera for distance estimation.")
    print("You'll need a person to stand at a measured distance from the camera.")
    print("The tool will capture multiple samples to calculate the focal length.")
    print("\n")
    
    try:
        known_distance = float(input("Enter the known distance to person (meters): "))
        known_height = float(input("Enter the person's height (meters) [default 1.7]: ") or "1.7")
    except ValueError:
        log_error("CALIBRATION", "Invalid input. Please enter numeric values.")
        return None
    
    # Capture and process frames
    samples = []
    
    log_info("CALIBRATION", "Starting camera feed for calibration")
    print("\n===== INSTRUCTIONS =====")
    print("1. Position a person at exactly", known_distance, "meters from camera")
    print("2. Make sure the full body is visible")
    print("3. Press 'c' to capture a sample")
    print("4. Capture at least 5 samples for accuracy")
    print("5. Press 'q' when done")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            log_error("CALIBRATION", "Failed to read frame")
            break
        
        # Run detection
        results = model(frame, imgsz=224, classes=[0], conf=args.confidence)
        
        # Display the frame with detections
        if len(results[0].boxes) > 0:
            # Get the largest person detection
            largest_box = None
            largest_area = 0
            
            for box in results[0].boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                area = (x2 - x1) * (y2 - y1)
                if area > largest_area:
                    largest_area = area
                    largest_box = (x1, y1, x2, y2)
            
            if largest_box:
                x1, y1, x2, y2 = largest_box
                # Draw the bounding box
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                box_height = y2 - y1
                cv2.putText(frame, f"Height: {box_height}px", (x1, y1-10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Display instructions and samples count
        cv2.putText(frame, f"Samples: {len(samples)}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(frame, "Press 'c' to capture, 'q' to finish", (10, 60),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        
        cv2.imshow("Distance Calibration", frame)
        key = cv2.waitKey(1)
        
        if key == ord('c') and len(results[0].boxes) > 0:
            # Take a sample using the largest person detection
            if largest_box:
                box_height = int(largest_box[3] - largest_box[1])
                samples.append(box_height)
                log_info("CALIBRATION", f"Sample captured: {box_height} pixels")
                print(f"Sample {len(samples)} captured: {box_height} pixels")
                
        elif key == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()
    
    if len(samples) < 3:
        log_error("CALIBRATION", "Not enough samples. Need at least 3 samples for reliable calibration.")
        return None
    
    # Calculate focal length using the median height to avoid outliers
    samples.sort()
    median_height = samples[len(samples) // 2]
    focal_length = (median_height * known_distance) / known_height
    
    # Calculate statistics
    mean_height = sum(samples) / len(samples)
    std_dev = (sum((x - mean_height) ** 2 for x in samples) / len(samples)) ** 0.5
    
    # Print results
    print("\n===== CALIBRATION RESULTS =====")
    print(f"Number of samples: {len(samples)}")
    print(f"Pixel heights: {samples}")
    print(f"Mean height: {mean_height:.2f} pixels")
    print(f"Median height: {median_height} pixels")
    print(f"Standard deviation: {std_dev:.2f} pixels")
    print(f"Calculated focal length: {focal_length:.2f}")
    print("\nTo use this value, update your settings.yaml file:")
    print("camera:")
    print(f"  focal_length: {focal_length:.2f}")
    print(f"  known_height: {known_height}")
    
    log_info("CALIBRATION", f"Calibration completed: Focal length = {focal_length:.2f}")
    return focal_length

if __name__ == "__main__":
    calibrate_distance()