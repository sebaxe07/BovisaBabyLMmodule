import zmq
import time
import random
from threading import Thread, Event, Lock
from utils.colored_logger import log_debug, log_info, log_error
import yaml
from ultralytics import YOLO
import cv2
from deep_sort_realtime.deepsort_tracker import DeepSort
import copy
import numpy as np
from boxmot import ByteTrack  # Faster alternative to DeepSORT
import torch
import queue

# filepath: /home/sebas/BovisaBabyLMmodule/sensors/camera_client.py

class CameraClient:
    def __init__(self, config):
        log_info("CAMERA", "Initializing camera client")
        self.config = config
        self.context = zmq.Context()
        self.running = False
        self.status = "STOPPED"
        self.streaming = False
        self.frame_queue = queue.Queue(maxsize=1)  # Store latest frame
        self.camera_running = False
        self.camera_lock = Lock()
        self.confidence_threshold = config['confidence_threshold']
        
        # FPS control parameters
        self.target_fps = config['target_fps']  # Target FPS for tracking
        self.streaming_fps = config['streaming_fps']  # Target FPS for streaming
        self.frame_time = 1.0 / self.target_fps  # Target time per frame in seconds
        self.streaming_frame_time = 1.0 / self.streaming_fps  # Target time for streaming mode
        self.fps_alpha = 0.1  # EMA smoothing factor for FPS calculation
        self.current_fps = 0  # Current smoothed FPS
        self.last_fps_log = 0  # Time of last FPS log
        log_info("CAMERA", f"Target FPS: {self.target_fps}, Streaming FPS: {self.streaming_fps}")
        
        # Frame processing thread events
        self.stream_event = Event()
        self.tracking_event = Event()
        
        # Default position values for tracking
        self.x_position = -5.0
        self.distance = 2.0
        self.human_id = 1
        
        # Initialize distance tracking
        self.distance_history = []
        self.max_history = 5  # Keep last 5 measurements
        self.id_tracked = 0

        # Initialize YOLO model
        torch.set_num_threads(4)  # Use all 4 Pi 5 cores
        self.model = YOLO(config['model'], task='pose')
        
        # Video stream publisher
        self.video_publisher = self.context.socket(zmq.PUB)
        self.video_publisher.bind("tcp://192.168.1.50:5559")
        log_info("CAMERA", "Video publisher initialized on port 5559")
        
        self.publisher = self.context.socket(zmq.PUB)
        self.publisher.bind("tcp://192.168.1.50:5558")
       
        # Setup communication sockets
        self.command_subscriber = self.context.socket(zmq.SUB)
        self.command_subscriber.connect("tcp://192.168.1.40:5557")
        self.command_subscriber.setsockopt_string(zmq.SUBSCRIBE, '')
        
        # Add synchronization delay
        time.sleep(0.5)
        log_info("CAMERA", "Waiting for publishers to be ready...")
        time.sleep(0.5)

        # Start the camera and processing threads
        self.start_camera()
        self.start_video_streaming()

        log_info("CAMERA", "Camera client initialized")
        
    def _init_camera(self, config):
        """Initialize camera parameters needed for tracking"""
        try:
            # Load camera configuration
            self.known_height = config['known_height']
            self.focal_length = config['focal_length']
            self.colors = config['colors']
            self.origin_point_bias = config['origin_point_bias']
            self.debug_visualization = config['debug_visualization']
            log_debug("CAMERA", f"Debug visualization: {self.debug_visualization}")
            
            # Initialize tracker
            self.tracker = ByteTrack()  # Initialize ByteTrack
            
            log_info("CAMERA", "Camera tracking parameters initialized successfully")
            
        except Exception as e:
            log_error("CAMERA", f"Error initializing camera tracking parameters: {e}")
            raise  # Re-raise to be caught in process_commands

    def start_camera(self):
        """Start the continuous camera capture thread"""
        if self.camera_running:
            log_info("CAMERA", "Camera is already running")
            return
            
        log_info("CAMERA", "Starting continuous camera capture")
        self.camera_running = True
        self.camera_thread = Thread(target=self._camera_capture_loop)
        self.camera_thread.daemon = True
        self.camera_thread.start()
        time.sleep(1.0)  # Give time for camera to initialize
        
    def stop_camera(self):
        """Stop the camera capture thread"""
        if not self.camera_running:
            return
            
        log_info("CAMERA", "Stopping camera capture")
        self.camera_running = False
        
        if hasattr(self, 'camera_thread') and self.camera_thread is not None:
            self.camera_thread.join(timeout=3.0)
            self.camera_thread = None
                
    def _camera_capture_loop(self):
        """Continuously capture frames from camera and put them in the queue"""
        log_info("CAMERA", "Starting camera capture loop")
        
        # Try to open the camera
        max_retries = 5
        retry_count = 0
        cap = None
        
        while retry_count < max_retries and not cap:
            try:
                cap = cv2.VideoCapture(0)
                if not cap.isOpened():
                    log_error("CAMERA", f"Failed to open camera on try {retry_count+1}/{max_retries}")
                    retry_count += 1
                    time.sleep(1.0)  # Wait before retry
                else:
                    break
            except Exception as e:
                log_error("CAMERA", f"Error opening camera on try {retry_count+1}: {e}")
                retry_count += 1
                time.sleep(1.0)  # Wait before retry
        
        if not cap or not cap.isOpened():
            log_error("CAMERA", f"Failed to open camera after {max_retries} attempts")
            return
            
        # Configure camera
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 224)
        log_info("CAMERA", "Camera successfully opened")
        
        # Store camera reference
        with self.camera_lock:
            self.cap = cap
        
        # FPS variables
        prev_capture_time = time.time()
        prev_fps_calc_time = time.time()
        frame_count = 0
        fps = 0
        
        # Tracking time difference between frames to maintain consistent FPS
        frame_times = []
        
        try:
            while self.camera_running:
                current_time = time.time()
                
                # Calculate elapsed time since last frame
                elapsed = current_time - prev_capture_time
                 
                # Determine target frame time based on current mode
                target_time = self.frame_time if self.running else self.streaming_frame_time
                
                # Wait if we're going too fast (respect the target framerate)
                if elapsed < target_time:
                    sleep_time = target_time - elapsed
                    time.sleep(sleep_time)
                    # Recalculate current time after sleep
                    current_time = time.time()
                
                # Read frame from camera
                ret, frame = cap.read()
                if not ret:
                    log_error("CAMERA", "Failed to read frame from camera")
                    time.sleep(0.5)
                    continue
                
                # Update capture time
                prev_capture_time = current_time
                
                # Update frame counter
                frame_count += 1
                
                # Calculate FPS every second
                if current_time - prev_fps_calc_time >= 1.0:
                    # Calculate actual FPS
                    fps = frame_count / (current_time - prev_fps_calc_time)
                    
                    # Apply EMA smoothing to FPS calculation
                    if self.current_fps == 0:  # First reading
                        self.current_fps = fps
                    else:
                        self.current_fps = (self.fps_alpha * fps) + ((1.0 - self.fps_alpha) * self.current_fps)
                    
                    # Log FPS periodically (every 5 seconds)
                    if current_time - self.last_fps_log >= 5.0:
                        mode = "TRACKING" if self.running else "STREAMING"
                        target = self.target_fps if self.running else self.streaming_fps
                        log_info("CAMERA", f"{mode} mode FPS: {self.current_fps:.1f} (target: {target})")
                        self.last_fps_log = current_time
                    
                    # Reset for next second
                    frame_count = 0
                    prev_fps_calc_time = current_time
                
                # Create a copy of the frame for processing
                frame_copy = frame.copy()
                
                # Store frame data with metadata
                frame_data = {
                    "frame": frame.copy(),  # Original frame without overlays
                    "display_frame": frame_copy,  # Frame without any text
                    "fps": self.current_fps,  # Use the smoothed FPS
                    "timestamp": time.time()
                }
                
                # Update queue with latest frame (replacing old frame if full)
                if self.frame_queue.full():
                    try:
                        self.frame_queue.get_nowait()
                    except queue.Empty:
                        pass
                self.frame_queue.put(frame_data)
                
        except Exception as e:
            log_error("CAMERA", f"Camera capture error: {e}")
            import traceback
            log_error("CAMERA", traceback.format_exc())
        
        finally:
            # Clean up
            with self.camera_lock:
                if cap and cap.isOpened():
                    cap.release()
                    self.cap = None
                    
        log_info("CAMERA", "Camera capture loop ended")

    def start_video_streaming(self):
        """Start continuous video streaming without detection"""
        if self.streaming:
            log_info("CAMERA", "Video streaming is already running")
            return
            
        log_info("CAMERA", "Starting video streaming")
        self.streaming = True
        self.stream_event.set()
        self.stream_thread = Thread(target=self._stream_video_loop)
        self.stream_thread.daemon = True
        self.stream_thread.start()
        
    def stop_video_streaming(self):
        """Stop the video streaming"""
        if not self.streaming:
            return
            
        log_info("CAMERA", "Stopping video streaming")
        self.streaming = False
        self.stream_event.clear()
        
        # Wait for thread to terminate with proper timeout
        if self.stream_thread:
            log_info("CAMERA", "Waiting for video streaming thread to terminate...")
            self.stream_thread.join(timeout=3.0)
            self.stream_thread = None

    def _stream_video_loop(self):
        """Continuous video streaming loop"""
        log_info("CAMERA", "Starting video streaming loop")
        
        while self.stream_event.is_set():
            try:
                # Skip processing if tracking is active to avoid duplicate frame sending
                if self.running:
                    time.sleep(0.1)  # Short sleep to check again soon
                    continue
                    
                # Get the latest frame
                try:
                    frame_data = self.frame_queue.get(timeout=1.0)
                    frame = frame_data["frame"]  # Get original frame without overlays
                    fps = frame_data["fps"]
                    
                    # Create a copy for adding UI elements
                    display_frame = frame.copy()
                    
                    # Add FPS and temperature info
                    cpu_temp = self.get_cpu_temperature()
                    cv2.putText(display_frame, f"FPS: {int(fps)}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.putText(display_frame, f"CPU: {cpu_temp:.1f}°C", (10, 70),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    
                    # Add status indicator (use consistent text)
                    cv2.putText(display_frame, "STREAMING MODE", (10, 110),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    
                    # Send the processed frame over ZMQ
                    self.send_frame(display_frame)
                    
                except queue.Empty:
                    # No frame available
                    time.sleep(0.1)
                    continue
                    
            except Exception as e:
                log_error("CAMERA", f"Video streaming error: {e}")
                import traceback
                log_error("CAMERA", traceback.format_exc())
                time.sleep(0.5)
            
            # Sleep to control streaming rate
            time.sleep(0.05)  # ~20fps when just streaming
                
        log_info("CAMERA", "Video streaming loop ended")

    def send_frame(self, frame):
        """Encode and send a frame over ZMQ for remote visualization"""
        try:
            # Encode the frame as JPEG
            _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            
            # Send the frame with metadata
            self.video_publisher.send_multipart([
                b"frame",
                buffer.tobytes(),
                f"{int(time.time() * 1000)}".encode('utf-8')  # Timestamp
            ])
        except Exception as e:
            log_error("CAMERA", f"Error sending frame: {e}")
   
    def start_tracking(self):
        """Start person tracking"""
        if self.running:
            log_info("CAMERA", "Tracking is already running")
            return
            
        log_info("CAMERA", "Starting tracking")
        
        try:
            # Initialize tracking parameters if not already done
            self._init_camera(self.config)
            
            # Start tracking
            self.running = True
            self.status = "SEARCHING"
            self.id_tracked = 0
            self.tracking_event.set()
            self.tracking_thread = Thread(target=self._tracking_loop)
            self.tracking_thread.daemon = True
            self.tracking_thread.start()
            
            log_info("CAMERA", "Tracking thread started")
            
        except Exception as e:
            log_error("CAMERA", f"Failed to start tracking: {e}")
            import traceback
            log_error("CAMERA", traceback.format_exc())
            self.running = False
            self.status = "STOPPED"
    
    def stop_tracking(self):
        """Stop tracking"""
        if not self.running:
            return
            
        log_info("CAMERA", "Stopping tracking")
        self.running = False
        self.tracking_event.clear()
        
        # Wait for the thread to terminate
        if self.tracking_thread:
            log_info("CAMERA", "Waiting for tracking thread to terminate...")
            self.tracking_thread.join(timeout=3.0)
            self.tracking_thread = None
            
        # Reset tracking data
        self.distance_history = []
        self.id_tracked = 0
        self.status = "STOPPED"
        
        log_info("CAMERA", "Tracking stopped")

    def cal_x_of_obj(self, position, frame_width=224):
        # Calculate the center of the rectangle
        x_center = (position[0] + position[2]) / 2
        
        # Get the frame center
        frame_center = frame_width / 2

        if frame_center == 0:
            log_error("CAMERA", "Frame center is zero, cannot calculate x position")
            return None
        
        # Calculate distance from center (negative = left, positive = right)
        offset_from_center = x_center - frame_center
        
        # Map to range -10 to 10
        scaled_position = (offset_from_center / frame_center) * 10
        
        return scaled_position
    
    def _tracking_loop(self):
        """Process frames for person tracking without controlling the camera"""
        log_info("CAMERA", "Starting tracking processing loop")
        
        # Initialize local variables for tracking
        min_distance = 10000
        min_ltrb = [1000] * 4
        min_track_id = 1000
        
        while self.tracking_event.is_set():
            try:
                # Get the latest frame
                try:
                    frame_data = self.frame_queue.get(timeout=1.0)
                    frame = frame_data["frame"]  # Get original frame without overlays
                    fps = frame_data["fps"]
                except queue.Empty:
                    # No frame available
                    time.sleep(0.1)
                    continue
                
                # Create a copy for processing and visualization
                display_frame = frame.copy()
                
                # Get frame dimensions
                frame_height, frame_width = frame.shape[:2]
                
                # Add FPS and temperature info
                cpu_temp = self.get_cpu_temperature()
                cv2.putText(display_frame, f"FPS: {int(fps)}", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.putText(display_frame, f"CPU: {cpu_temp:.1f}°C", (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                # Add status indicator 
                cv2.putText(display_frame, "TRACKING MODE", (10, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                
                # Run YOLO Detection
                results = self.model(frame, imgsz=224, conf=self.confidence_threshold, verbose=False)
                
                # Reset tracking variables for this frame
                min_distance = 10000
                min_ltrb = [1000] * 4
                min_track_id = 1000
                
                if len(results[0].keypoints) > 0:
                    detections = []
                    for i, kpts in enumerate(results[0].keypoints.data):
                        # Calculate bounding box from keypoints
                        valid_kpts = kpts[kpts[:, 2] > 0.5]  # Only use keypoints with confidence > 0.5
                        if len(valid_kpts) > 0:
                            x1, y1 = valid_kpts[:, 0].min(), valid_kpts[:, 1].min()
                            x2, y2 = valid_kpts[:, 0].max(), valid_kpts[:, 1].max()
                            x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
                            conf = float(valid_kpts[:, 2].mean())  # Average confidence
                            detections.append([x1, y1, x2, y2, conf, 0])  # cls=0 for person

                    # Convert to numpy array for ByteTrack
                    detections_np = np.array(detections)

                    if len(detections_np) > 0: 
                        try: 
                            # ByteTrack Update
                            tracks = self.tracker.update(detections_np, frame)

                            for track in tracks:
                                try:
                                    # ByteTrack returns [x1, y1, x2, y2, track_id, conf, cls, ...]
                                    x1, y1, x2, y2, track_id = map(int, track[:5])
                                    ltrb = [x1, y1, x2, y2]
                                    
                                    # Calculate distance
                                    keypoints_for_track = None
                                    for i, kpts in enumerate(results[0].keypoints.data):
                                        # Find keypoints that match this track's bounding box
                                        kpts_x_center = float(kpts[:, 0].mean())
                                        kpts_y_center = float(kpts[:, 1].mean())
                                        if (x1 <= kpts_x_center <= x2 and y1 <= kpts_y_center <= y2):
                                            keypoints_for_track = kpts
                                            break

                                    # Calculate distance with keypoints
                                    box_height = y2 - y1
                                    if keypoints_for_track is not None:
                                        distance, correction_applied = self.estimate_distance_from_keypoints(
                                            keypoints_for_track, box_height, y1, y2, frame_height)
                                    else:
                                        distance, correction_applied = self.estimate_distance(box_height, y1, y2, frame_height)

                                    # Apply smoothing filter
                                    self.distance_history.append(distance)
                                    while len(self.distance_history) > self.max_history:
                                        self.distance_history.pop(0)

                                    # Use median for stability (more robust to outliers than average)
                                    if len(self.distance_history) > 1:
                                        distance = sorted(self.distance_history)[len(self.distance_history)//2]
                                    
                                    # Add visual indicator for partial visibility if debugging
                                    if self.debug_visualization and correction_applied:
                                        # Add a "PARTIAL" indicator at the top of the bounding box
                                        cv2.putText(display_frame, "PARTIAL", (x1, y1+10), 
                                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                                    
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

                                    if self.debug_visualization:
                                        for i, kpts in enumerate(results[0].keypoints.data):
                                            # Draw keypoints
                                            for kp in kpts:
                                                x, y, conf = kp
                                                if conf > 0.5:  # Only draw keypoints with confidence > 0.5
                                                    cv2.circle(display_frame, (int(x), int(y)), 3, (0, 255, 0), -1)
                                            
                                            # Draw connections between keypoints (skeleton)
                                            # COCO keypoint connections
                                            skeleton = [
                                                [16, 14], [14, 12], [17, 15], [15, 13], [12, 13], [6, 12], [7, 13], 
                                                [6, 7], [6, 8], [7, 9], [8, 10], [9, 11], [2, 3], [1, 2], [1, 3], 
                                                [2, 4], [3, 5], [4, 6], [5, 7]
                                            ]
                                            
                                            for connection in skeleton:
                                                idx1, idx2 = connection
                                                try:
                                                    # Get the connection points
                                                    con_pts = kpts[[idx1-1, idx2-1]]
                                                    # Convert tensor min to scalar before boolean comparison
                                                    min_conf = float(con_pts[:, 2].min())  # Explicit conversion to float
                                                    if min_conf > 0.5:  # Now a simple float comparison
                                                        pt1 = (int(con_pts[0, 0]), int(con_pts[0, 1]))
                                                        pt2 = (int(con_pts[1, 0]), int(con_pts[1, 1]))
                                                        cv2.line(display_frame, pt1, pt2, (255, 0, 0), 2)
                                                except Exception as e:
                                                    # Skip problematic connections silently
                                                    continue
                                            
                                            # Draw bounding box for reference
                                            color = self.colors[track_id % len(self.colors)]
                                            cv2.rectangle(display_frame, (x1, y1), (x2, y2), color, 1)  # Thinner box

                                    if self.debug_visualization:
                                        # Draw bounding box and label
                                        color = self.colors[track_id % len(self.colors)]
                                        cv2.putText(display_frame,
                                                f"Person {track_id} ({distance:.2f}m)",
                                                (x1, y1-10),
                                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                                        
                                        # Calculate center point
                                        center_x = (x1 + x2) // 2
                                        center_y = (y1 + y2) // 2
                                        
                                        # Calculate x value using cal_x_of_obj
                                        x_value = self.cal_x_of_obj(ltrb, frame_width)
                                        
                                        if x_value is None:
                                            log_error("CAMERA", "Failed to calculate x position")
                                            break

                                        # Draw center point
                                        cv2.circle(display_frame, (center_x, center_y), 5, (0, 0, 255), -1)  # Red filled circle
                                        
                                        # Draw x value
                                        cv2.putText(display_frame,
                                                f"x: {x_value:.1f}",
                                                (center_x + 10, center_y),
                                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                                        
                                        # Draw LTRB values at respective corners
                                        font_scale = 0.4
                                        font_thickness = 1
                                        # Left
                                        cv2.putText(display_frame, f"L:{x1}", (x1-5, center_y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 0), font_thickness)
                                        # Top
                                        cv2.putText(display_frame, f"T:{y1}", (center_x, y1-5), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 0), font_thickness)
                                        # Right
                                        cv2.putText(display_frame, f"R:{x2}", (x2+5, center_y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 0), font_thickness)
                                        # Bottom
                                        cv2.putText(display_frame, f"B:{y2}", (center_x, y2+15), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 0), font_thickness)

                                except Exception as e:
                                    # If any error occurs, reinitialize the tracker
                                    self.tracker = ByteTrack() 
                                    log_error("CAMERA", f"Tracking error: {e}")
                                    message = {
                                        "type": "NOTFOUND",
                                        "x_position": 0,
                                        "distance": 0,
                                        "human_id": 0
                                    }
                                    # Assume we lost the track
                                    self.id_tracked = 0
                                    min_distance = 10000
                                    continue
                    
                        except np.linalg.LinAlgError as e:
                            # Handle numerical stability errors from ByteTrack
                            log_error("CAMERA", f"ByteTrack numerical stability error: {e}")
                            # Reset tracker on severe error
                            self.tracker = ByteTrack()
                            continue
                        except Exception as e:
                            # Handle any other exceptions
                            log_error("CAMERA", f"Tracking loop error: {e}")
                            import traceback
                            log_error("CAMERA", traceback.format_exc())

                # If no tracks are found or none matched our criteria
                if min_distance == 10000 and self.status == "TRACKING":
                    # We lost the track
                    log_error("CAMERA", "Track lost")
                    message = {
                        "type": "NOTFOUND",
                        "x_position": 0,
                        "distance": 0,
                        "human_id": 0
                    }
                    self.id_tracked = 0
                    self.status = "LOST"
                    self.publisher.send_json(message)
                elif min_distance == 10000 and self.status == "SEARCHING":
                    # No track found, but we are already searching
                    message = {
                        "type": "NOTFOUND",
                        "x_position": 0,
                        "distance": 0,
                        "human_id": 0
                    }
                    self.id_tracked = 0
                    self.publisher.send_json(message)
                else:
                    # We found a track
                    self.status = "TRACKING"
                    x_value = self.cal_x_of_obj(min_ltrb, frame_width)
                    if x_value is None:
                        log_error("CAMERA", "Failed to calculate x position")
                        continue

                    message = {
                        "type": "TRACKING",
                        "x_position": x_value,
                        "distance": min_distance,
                        "human_id": min_track_id
                    }
                    self.id_tracked = min_track_id
                    self.publisher.send_json(message)

                # Send the frame with tracking visualizations
                self.send_frame(display_frame)
                
            except Exception as e:
                log_error("CAMERA", f"Tracking process error: {e}")
                import traceback
                log_error("CAMERA", traceback.format_exc())
                time.sleep(0.5)
            
            # Process at a slightly higher rate when tracking
            time.sleep(0.02)  # ~50fps desired when tracking
                
        log_info("CAMERA", "Tracking loop ended")
        
    def estimate_distance(self, box_height, y1, y2, frame_height=None):
        """
        Estimate distance with compensation for partially visible humans
        
        Args:
            box_height: Height of the bounding box in pixels
            y1: Top y-coordinate of the bounding box
            y2: Bottom y-coordinate of the bounding box
            frame_height: Height of the camera frame
        """
        # Check if the person is touching bottom edge
        bottom_margin = 10  # Pixels from bottom to consider as "touching edge"
        is_cut_off_bottom = (frame_height - y2 <= bottom_margin)
        
        # Standard calculation for fully visible person
        if not is_cut_off_bottom:
            return (self.known_height * self.focal_length) / box_height, False
        
        # Person is cut off at bottom - apply correction
        
        # Estimate visible portion based on typical human body proportions
        # Full standing human proportions: head+torso ~68% of height, legs ~32%
        # We use y1 position to estimate how much of person is likely in frame
        
        # Check how high up the detection starts (lower y1 = more of upper body visible)
        relative_y1_pos = y1 / frame_height
        
        if relative_y1_pos < 0.2:
            # Mostly complete upper body visible - assume 68% visibility
            visible_ratio = 0.68
        elif relative_y1_pos < 0.4:
            # Partial upper body - assume 50% visibility
            visible_ratio = 0.50
        else:
            # Only upper portion visible - assume 35% visibility
            visible_ratio = 0.35
            
        # Apply correction
        corrected_height = box_height / visible_ratio
        distance = (self.known_height * self.focal_length) / corrected_height
        
        return distance, True

    def estimate_distance_from_keypoints(self, keypoints, box_height, y1, y2, frame_height=None):
        # YOLOv8 pose keypoint indices
        # 0: nose, 5-6: shoulders, 11-12: hips, 15-16: ankles
        
        # Define confidence threshold for valid keypoints
        conf_threshold = 0.5
        
        try:
            # Extract key anatomical points with confidence above threshold
            # Convert tensors to native Python types BEFORE any comparison
            nose = None
            shoulders = [None, None]
            hips = [None, None]
            ankles = [None, None]
            
            # Safely process nose keypoint
            try:
                if keypoints[0].shape[0] == 3:  # Ensure it has 3 elements (x,y,conf)
                    nose_conf = float(keypoints[0][2])
                    if nose_conf > conf_threshold:
                        nose = keypoints[0].cpu().numpy().copy() if hasattr(keypoints[0], 'cpu') else keypoints[0].copy()
            except (IndexError, AttributeError, ValueError) as e:
                log_debug("CAMERA", f"Error processing nose keypoint: {e}")
            
            # Process shoulder keypoints
            keypoint_pairs = [
                (5, 6, shoulders),  # shoulders
                (11, 12, hips),     # hips
                (15, 16, ankles)    # ankles
            ]
            
            for idx1, idx2, point_list in keypoint_pairs:
                try:
                    # First point
                    if idx1 < len(keypoints) and keypoints[idx1].shape[0] == 3:
                        conf = float(keypoints[idx1][2])
                        if conf > conf_threshold:
                            point_list[0] = keypoints[idx1].cpu().numpy().copy() if hasattr(keypoints[idx1], 'cpu') else keypoints[idx1].copy()
                    
                    # Second point
                    if idx2 < len(keypoints) and keypoints[idx2].shape[0] == 3:
                        conf = float(keypoints[idx2][2])
                        if conf > conf_threshold:
                            point_list[1] = keypoints[idx2].cpu().numpy().copy() if hasattr(keypoints[idx2], 'cpu') else keypoints[idx2].copy()
                except (IndexError, AttributeError, ValueError) as e:
                    log_debug("CAMERA", f"Error processing keypoint pair {idx1},{idx2}: {e}")

            # Check which body parts are visible
            has_head = nose is not None
            has_shoulders = any(s is not None for s in shoulders)
            has_hips = any(h is not None for h in hips)
            has_ankles = any(a is not None for a in ankles)

            # Full body visible - use ankle to head distance
            if has_head and has_ankles:
                # Get highest ankle y-position
                ankle_y = min([float(a[1]) for a in ankles if a is not None])
                head_y = float(nose[1])
                height_pixels = ankle_y - head_y
                # Person's full height is visible
                return (self.known_height * self.focal_length) / height_pixels, False
                
            # Upper body visible (to hips) - use head to hip ratio
            elif has_head and has_hips:
                # Get lowest hip y-position
                hip_y = max([float(h[1]) for h in hips if h is not None])
                head_y = float(nose[1])
                torso_height_pixels = hip_y - head_y
                estimated_height_pixels = torso_height_pixels / 0.46
                return (self.known_height * self.focal_length) / estimated_height_pixels, True
                
            # Only upper torso visible - use head to shoulder ratio
            elif has_head and has_shoulders:
                # Get lowest shoulder y-position
                shoulder_y = max([float(s[1]) for s in shoulders if s is not None])
                head_y = float(nose[1])
                upper_torso_pixels = shoulder_y - head_y
                # Upper torso is ~20% of total height (refined from 18%)
                estimated_height_pixels = upper_torso_pixels / 0.20
                return (self.known_height * self.focal_length) / estimated_height_pixels, True
            
            # Fallback to bounding box with correction
            else:
                # Use your existing method as fallback
                log_debug("CAMERA", "Fallback to bounding box distance estimation")
                return self.estimate_distance(box_height, y1, y2, frame_height)
            
        except Exception as e:
            import traceback
            log_error("CAMERA", f"Keypoint distance estimation error: {e}")
            log_error("CAMERA", traceback.format_exc())
            # Fallback to standard estimation
            return self.estimate_distance(box_height, y1, y2, frame_height)
            
    def get_cpu_temperature(self):
        """Get the Raspberry Pi CPU temperature in Celsius"""
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as temp_file:
                temp = float(temp_file.read().strip()) / 1000
            return temp
        except Exception as e:
            log_error("CAMERA", f"Failed to read CPU temperature: {e}")
            return 0.0
        
    def process_commands(self):
        """Process incoming commands and control the camera"""
        log_info("CAMERA", "Starting command processing loop")

        while True:
            try:
                # Receive command
                cmd = self.command_subscriber.recv_json(zmq.NOBLOCK)
                log_info("CAMERA", f"Received command: {cmd['command']}")

                if cmd['command'] == "SEARCH":
                    if not self.running:
                        # Start tracking mode (detection processing)
                        self.start_tracking()
                        
                elif cmd['command'] == "STOP":
                    if self.running:
                        # Stop tracking mode (but keep camera running)
                        self.stop_tracking()

            except zmq.Again:
                pass

            time.sleep(0.01)

    def cleanup(self):
        """Clean up resources before exit"""
        log_info("CAMERA", "Cleaning up resources")
        
        # Stop tracking if running
        if self.running:
            self.stop_tracking()
        
        # Stop video streaming
        if self.streaming:
            self.stop_video_streaming()
            
        # Stop camera capture
        if self.camera_running:
            self.stop_camera()
            
        # Close ZMQ resources
        if hasattr(self, 'video_publisher'):
            self.video_publisher.close()
            
        if hasattr(self, 'publisher'):
            self.publisher.close()
            
        if hasattr(self, 'command_subscriber'):
            self.command_subscriber.close()
            
        self.context.term()