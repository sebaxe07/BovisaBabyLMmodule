import zmq
import time
import random
from threading import Thread
from utils.colored_logger import log_debug, log_info, log_error
import yaml
from ultralytics import YOLO
import cv2
from deep_sort_realtime.deepsort_tracker import DeepSort
import copy
import numpy as np
from boxmot import ByteTrack  # Faster alternative to DeepSORT
import torch

# filepath: /home/sebas/BovisaBabyLMmodule/sensors/camera_client.py

class MockCameraClient:
    def __init__(self, config):
        log_info("CAMERA", "Initializing mock camera client")
        self.config = config
        self.context = zmq.Context()
        self.running = False


        # Load camera configuration
        self.known_height = config['known_height']
        self.focal_length = config['focal_length']
        self.colors = config['colors']
        self.origin_point_bias = config['origin_point_bias']
        self.debug_visualization = config['debug_visualization']


        torch.set_num_threads(4)  # Use all 4 Pi 5 cores

        # Load camera parameters
        self.model = YOLO('yolov8n.onnx', task='detect')
        self.tracker = ByteTrack()  # Initialize ByteTrack
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 224) 
        self.confidence_threshold = config['confidence_threshold']

        self.id_tracked = 0

        # Default position values for mock camera
        self.x_position = -5.0
        self.distance = 2.0
        self.human_id = 1
        
        self.publisher = self.context.socket(zmq.PUB)
        self.publisher.bind("tcp://*:5558")
       
        # Setup communication sockets
        self.command_subscriber = self.context.socket(zmq.SUB)
        self.command_subscriber.connect("tcp://localhost:5557")
        self.command_subscriber.setsockopt_string(zmq.SUBSCRIBE, '')
        
        # Add synchronization delay or handshake
        time.sleep(0.5)
        log_info("CAMERA", "Waiting for publishers to be ready...")
        time.sleep(0.5)



        log_info("CAMERA", "Mock camera client initialized")

    def _generate_tracking_data(self):
        """Generate mock tracking data"""
        while self.running:
            # Use the current position values
            # Create message
            message = {
                "type": "TRACKING",
                "x_position": self.x_position,
                "distance": self.distance,
                "human_id": self.human_id
            }

            # Send message
            self.publisher.send_json(message)
            log_info("CAMERA", f"Sent tracking data: x={self.x_position}, dist={self.distance}")

            # Wait before sending the next message
            time.sleep(0.5)

    def cal_x_of_obj(self, position):
        x_center = (position[2] - position[0])/2 - self.origin_point_bias[0]
        y_center = (position[2] - position[0])/2 - self.origin_point_bias[1]
        return x_center


    def estimate_distance(self, box_height):
        return (self.known_height * self.focal_length) / box_height
        
    def get_cpu_temperature(self):
        """Get the Raspberry Pi CPU temperature in Celsius"""
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as temp_file:
                temp = float(temp_file.read().strip()) / 1000
            return temp
        except Exception as e:
            log_error("CAMERA", f"Failed to read CPU temperature: {e}")
            return 0.0
    

    def _getting_tracking_data(self):
        """Getting tracking data from the camera using ByteTrack"""
        log_info("CAMERA", "Starting tracking loop with ByteTrack")
        self.id_tracked = 0

        # FPS variables
        prev_time = 0
        fps = 0
        
        while self.running:
            # Read frame from camera
            ret, frame = self.cap.read()
            if not ret:
                log_error("CAMERA", "Failed to read frame from camera")
                break

            # Calculate FPS
            current_time = time.time()
            fps = 1 / (current_time - prev_time)
            prev_time = current_time

            # YOLO Detection
            results = self.model(frame, imgsz=224, classes=[0], conf=self.confidence_threshold, verbose=False)
            detections = []

            if len(results[0].boxes) > 0:
                # Prepare detections in ByteTrack format: [x1, y1, x2, y2, conf, cls]
                for box in results[0].boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    conf = float(box.conf[0])
                    detections.append([x1, y1, x2, y2, conf, 0])  # cls=0 for person

                # Convert to numpy array for ByteTrack
                detections_np = np.array(detections)
                
                # ByteTrack Update
                tracks = self.tracker.update(detections_np, frame)

                min_distance = 10000
                min_ltrb = [1000] * 4
                min_track_id = 1000

                for track in tracks:
                    try:
                        # ByteTrack returns [x1, y1, x2, y2, track_id, conf, cls, ...]
                        x1, y1, x2, y2, track_id = map(int, track[:5])
                        ltrb = [x1, y1, x2, y2]
                        
                        # Calculate distance
                        box_height = y2 - y1
                        distance = self.estimate_distance(box_height)
                        
                        # Check if we are tracking a person
                        if self.id_tracked == 0:
                            # We are not tracking a person, find the closest one
                            if min_distance > distance:
                                min_distance = distance
                                min_ltrb = copy.copy(ltrb)
                                min_track_id = track_id
                        else:
                            # We are already tracking a person, check if it's the same one
                            if track_id == self.id_tracked:
                                min_distance = distance
                                min_ltrb = copy.copy(ltrb)
                                min_track_id = track_id

                        log_debug("CAMERA", f"Track ID: {track_id}, Distance: {distance:.2f}m, LTRB: {ltrb}")
                        
                        if self.debug_visualization:
                            # Draw bounding box and label
                            color = self.colors[track_id % len(self.colors)]
                            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                            cv2.putText(frame,
                                    f"Person {track_id} ({distance:.2f}m)",
                                    (x1, y1-10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                    except Exception as e:
                        log_error("CAMERA", f"Tracking error: {e}")
                        message = {
                            "type": "NOTFOUND",
                            "x_position": 0,
                            "distance": 0,
                            "human_id": 0
                        }
                        # Assume we lost the track
                        self.id_tracked = 0
                        continue

                # If no tracks are found or none matched our criteria
                if min_distance == 10000:
                    log_error("CAMERA", "No tracks found or track lost")
                    message = {
                        "type": "NOTFOUND",
                        "x_position": 0,
                        "distance": 0,
                        "human_id": 0
                    }
                    self.id_tracked = 0
                else:
                    # We found a track
                    message = {
                        "type": "TRACKING",
                        "x_position": self.cal_x_of_obj(min_ltrb),
                        "distance": min_distance,
                        "human_id": min_track_id
                    }
                    log_info("CAMERA", f"Found track: {min_track_id} at distance {min_distance:.2f}m at position {self.cal_x_of_obj(min_ltrb)}")
                    self.id_tracked = min_track_id

            else:
                message = {
                    "type": "NOTFOUND",
                    "x_position": 0,
                    "distance": 0,
                    "human_id": 0
                } 
                self.id_tracked = 0

            if self.debug_visualization:
                # Display FPS on frame
                # Get CPU temperature
                cpu_temp = self.get_cpu_temperature()
    
                # Display FPS and temperature on frame
                cv2.putText(frame, f"FPS: {int(fps)}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
                cv2.putText(frame, f"CPU: {cpu_temp:.1f}°C", (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                cv2.imshow("Human Tracking", frame)
                if cv2.waitKey(1) == ord('q'):
                    break
            else:
                 log_info("CAMERA", f"FPS: {int(fps)}, CPU: {self.get_cpu_temperature():.1f}°C")          
                 
            # Send message
            self.publisher.send_json(message)
            log_info("CAMERA", f"Sent tracking data: x={message['x_position']}, dist={message['distance']}, human_id={message['human_id']}")

            # Wait before sending the next message
            target_fps = 10  # Adjust based on your needs
            frame_time = 1.0 / target_fps
            processing_time = time.time() - current_time
            sleep_time = max(0, frame_time - processing_time)
            time.sleep(sleep_time)

        if self.debug_visualization:
            cv2.destroyAllWindows()
        log_info("CAMERA", "Tracking loop ended")

    def process_commands(self):
        """Process incoming commands and control the mock camera"""
        log_info("CAMERA", "Starting command processing loop")
        while True:
            try:
                # Receive command
                cmd = self.command_subscriber.recv_json(zmq.NOBLOCK)
                log_info("CAMERA", f"Received command: {cmd['command']}")

                if cmd['command'] == "SEARCH":
                    if not self.running:
                        log_info("CAMERA", "Starting tracking")
                        self.running = True
                        self.tracking_thread = Thread(target=self._getting_tracking_data)
                        self.tracking_thread.daemon = True
                        self.tracking_thread.start()

                elif cmd['command'] == "STOP":
                    if self.running:
                        log_info("CAMERA", "Stopping tracking")
                        self.running = False
                        # Release camera resources
                        self.cap.release()
                        self.tracking_thread.join(timeout=2.0)  # Add timeout for safety

                
                elif cmd['command'] == "UPDATE_HUMAN_POSITION":
                    # Update position values
                    self.x_position = cmd.get('x_position', self.x_position)
                    self.distance = cmd.get('distance', self.distance)
                    log_info("CAMERA", f"Updated position: x={self.x_position}, distance={self.distance}")

            except zmq.Again:
                pass

            time.sleep(0.01)

    def cleanup(self):
        """Clean up resources before exit"""
        log_info("CAMERA", "Cleaning up resources")
        self.running = False
        self.cap.release()
        if self.debug_visualization:
            cv2.destroyAllWindows()
        self.context.term()