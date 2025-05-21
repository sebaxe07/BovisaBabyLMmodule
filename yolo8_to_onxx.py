from ultralytics import YOLO

# Load model and export to ONNX with 320x320 input
model = YOLO('yolov8n-pose.pt')
model.export(format='onnx', imgsz=480)  # Force 320x320 input shape