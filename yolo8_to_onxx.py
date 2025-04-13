from ultralytics import YOLO

# Load model and export to ONNX with 320x320 input
model = YOLO('yolov8n.pt')
model.export(format='onnx', imgsz=224)  # Force 320x320 input shape