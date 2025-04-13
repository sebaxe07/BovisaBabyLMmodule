from ultralytics import YOLO
import cv2
import time
import numpy as np
from boxmot import ByteTrack  # Faster alternative to DeepSORT

# Load YOLOv8 Nano ONNX model
model = YOLO('yolov8n.onnx', task='detect')
tracker = ByteTrack()  # Initialize ByteTrack

# Webcam setup
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

# FPS variables
prev_time = 0
fps = 0

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Calculate FPS
    current_time = time.time()
    fps = 1 / (current_time - prev_time)
    prev_time = current_time

    # YOLOv8 inference (optimized for speed)
    results = model(frame, imgsz=224, conf=0.4, classes=[0], verbose=False)
    
    # Convert detections to NumPy array (ByteTrack format: [x1, y1, x2, y2, conf, cls])
    detections = []
    for box in results[0].boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        conf = float(box.conf)
        cls = int(box.cls)
        detections.append([x1, y1, x2, y2, conf, cls])
    
    detections = np.array(detections)  # Convert to NumPy array

    # Update tracker (only if detections exist)
    if len(detections) > 0:
        tracks = tracker.update(detections, frame)  # ByteTrack requires NumPy input
    else:
        tracks = []

    # Draw tracked objects
    for track in tracks:
        x1, y1, x2, y2, track_id = map(int, track[:5])
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f"ID: {track_id}", (x1, y1-10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Display FPS
    cv2.putText(frame, f"FPS: {int(fps)}", (10, 30), 
               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    cv2.imshow('YOLOv8 + ByteTrack', frame)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()