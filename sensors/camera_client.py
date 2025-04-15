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

class CameraClient:
    def __init__(self, config):
        log_info("CAMERA", "Initializing camera client")
        self.config = config
        self.context = zmq.Context()
        self.running = False
        self.status = "STOPPED"

        torch.set_num_threads(4)  # Use all 4 Pi 5 cores
        self.model = YOLO(config['model'], task='pose')

        # Default position values for mock camera
        self.x_position = -5.0
        self.distance = 2.0
        self.human_id = 1
        
        # Video stream publisher
        self.video_publisher = self.context.socket(zmq.PUB)
        self.video_publisher.bind("tcp://*:5559")  # New port for video stream
        log_info("CAMERA", "Video publisher initialized on port 5559")
        
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



        log_info("CAMERA", "Camera client initialized")
        
    def _init_camera(self, config):
        try:
            # Load camera configuration
            self.known_height = config['known_height']
            self.focal_length = config['focal_length']
            self.colors = config['colors']
            self.origin_point_bias = config['origin_point_bias']
            self.debug_visualization = config['debug_visualization']
            log_debug("CAMERA", f"Debug visualization: {self.debug_visualization}")
            self.distance_history = []
            self.max_history = 5  # Keep last 5 measurements

            # Release camera if it's already open
            if hasattr(self, 'cap') and self.cap is not None:
                try:
                    self.cap.release()
                    time.sleep(0.5)  # Give it time to fully release
                except:
                    pass

            # Initialize tracker
            self.tracker = ByteTrack()  # Initialize ByteTrack
            
            # Try to open camera
            self.cap = cv2.VideoCapture(0)
            if not self.cap.isOpened():
                raise Exception("Failed to open camera")
                
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 224) 
            self.confidence_threshold = config['confidence_threshold']
            self.id_tracked = 0
            
            log_info("CAMERA", "Camera initialized successfully")
            
        except Exception as e:
            log_error("CAMERA", f"Error initializing camera: {e}")
            raise  # Re-raise to be caught in process_commands


    def send_frame(self, frame):
        """Encode and send a frame over ZMQ for remote visualization"""
        try:
            # Resize for network efficiency if needed
            # target_width = 640 
            # target_height = int(frame.shape[0] * (target_width / frame.shape[1]))
            # frame = cv2.resize(frame, (target_width, target_height))
            
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
            #log_info("CAMERA", f"Sent tracking data: x={self.x_position}, dist={self.distance}")

            # Wait before sending the next message
            time.sleep(0.5)

    def cal_x_of_obj(self, position):
        # Calculate the center of the rectangle
        x_center = (position[0] + position[2]) / 2
        
        # Get the frame center
        frame_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        frame_center = frame_width / 2

        if frame_center == 0:
            log_error("CAMERA", "Frame center is zero, cannot calculate x position")
            return None
        
        # Calculate distance from center (negative = left, positive = right)
        offset_from_center = x_center - frame_center
        
        # Map to range -10 to 10

        scaled_position = (offset_from_center / frame_center) * 10
        
        return scaled_position

    def estimate_distance(self, box_height, y1, y2, frame_height=None):
        """
        Estimate distance with compensation for partially visible humans
        
        Args:
            box_height: Height of the bounding box in pixels
            y1: Top y-coordinate of the bounding box
            y2: Bottom y-coordinate of the bounding box
            frame_height: Height of the camera frame
        """
        # Get the frame height if not provided
        if frame_height is None:
            frame_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        
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
            
            # Get frame height if not provided
            if frame_height is None:
                frame_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            

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
    

    def _getting_tracking_data(self):
        """Getting tracking data from the camera using ByteTrack"""
        log_info("CAMERA", "Starting tracking loop with ByteTrack")
        self.id_tracked = 0

        # FPS variables
        prev_time = 0
        fps = 0

        try: 
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
                results = self.model(frame, imgsz=224, conf=self.confidence_threshold, verbose=False)
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
                    min_distance = 10000
                    min_ltrb = [1000] * 4
                    min_track_id = 1000

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
                                        kpts_x_center = float(kpts[:, 0].mean())  # Add float() here
                                        kpts_y_center = float(kpts[:, 1].mean())  # Add float() here
                                        if (x1 <= kpts_x_center <= x2 and y1 <= kpts_y_center <= y2):
                                            keypoints_for_track = kpts
                                            break

                                    # Calculate distance with keypoints
                                    box_height = y2 - y1
                                    frame_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
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
                                        cv2.putText(frame, "PARTIAL", (x1, y1+10), 
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
                                                        cv2.circle(frame, (int(x), int(y)), 3, (0, 255, 0), -1)
                                                
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
                                                            cv2.line(frame, pt1, pt2, (255, 0, 0), 2)
                                                    except Exception as e:
                                                        # Skip problematic connections silently
                                                        continue
                                                # Still draw bounding box for reference
                                                x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
                                                color = self.colors[track_id % len(self.colors)] if 'track_id' in locals() else (0, 255, 0)
                                                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 1)  # Thinner box

                                    if self.debug_visualization:
                                        # Draw bounding box and label
                                        color = self.colors[track_id % len(self.colors)]
                                        cv2.putText(frame,
                                                f"Person {track_id} ({distance:.2f}m)",
                                                (x1, y1-10),
                                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                                        
                                        # Calculate center point
                                        center_x = (x1 + x2) // 2
                                        center_y = (y1 + y2) // 2
                                        
                                        # Calculate x value using cal_x_of_obj
                                        x_value = self.cal_x_of_obj(ltrb)
                                        
                                        if x_value is None:
                                            log_error("CAMERA", "Failed to calculate x position")
                                            break

                                        # Draw center point
                                        cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)  # Red filled circle
                                        
                                        # Draw x value
                                        cv2.putText(frame,
                                                f"x: {x_value:.1f}",
                                                (center_x + 10, center_y),
                                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                                        
                                        # Draw LTRB values at respective corners
                                        font_scale = 0.4
                                        font_thickness = 1
                                        # Left
                                        cv2.putText(frame, f"L:{x1}", (x1-5, center_y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 0), font_thickness)
                                        # Top
                                        cv2.putText(frame, f"T:{y1}", (center_x, y1-5), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 0), font_thickness)
                                        # Right
                                        cv2.putText(frame, f"R:{x2}", (x2+5, center_y), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 0), font_thickness)
                                        # Bottom
                                        cv2.putText(frame, f"B:{y2}", (center_x, y2+15), cv2.FONT_HERSHEY_SIMPLEX, font_scale, (255, 255, 0), font_thickness)

                                except Exception as e:
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
                            log_error("CAMERA", f"ByteTrack numerical stability error: {e}")
                            # Reset tracker on severe error
                            self.tracker = ByteTrack()
                            continue
                        except Exception as e:
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
                    elif min_distance == 10000 and self.status == "SEARCHING":
                        # No track found, but we are already searching
                        message = {
                            "type": "NOTFOUND",
                            "x_position": 0,
                            "distance": 0,
                            "human_id": 0
                        }
                        self.id_tracked = 0
                    else:
                        # We found a track
                        self.status = "TRACKING"
                        x_value = self.cal_x_of_obj(min_ltrb)
                        if x_value is None:
                            log_error("CAMERA", "Failed to calculate x position")
                            break

                        message = {
                            "type": "TRACKING",
                            "x_position": x_value,
                            "distance": min_distance,
                            "human_id": min_track_id
                        }
                        # log_info("CAMERA", f"Found track: {min_track_id} at distance {min_distance:.2f}m at position {x_value:.1f}")
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

                    #cv2.imshow("Human Tracking", frame)
                    self.send_frame(frame)  # Send frame to publisher

                    if cv2.waitKey(1) == ord('q'):
                        log_debug("CAMERA", "Exiting debug visualization loop")
                        self.running = False
                        break
                #else:
                    #log_info("CAMERA", f"FPS: {int(fps)}, CPU: {self.get_cpu_temperature():.1f}°C")          
                    
                # Send message to publisher one time if we lost the track or constant updates if we are tracking
                if self.status == "LOST":
                    # Send message
                    self.publisher.send_json(message)
                    #log_info("CAMERA", f"Sent tracking data: x={message['x_position']}, dist={message['distance']}, human_id={message['human_id']}")
                    # Reset status
                    self.status = "SEARCHING"
                elif self.status == "TRACKING":
                    self.publisher.send_json(message)
                    #log_info("CAMERA", f"Sent tracking data: x={message['x_position']}, dist={message['distance']}, human_id={message['human_id']}")

                # Wait before sending the next message
                # target_fps = 10  # Adjust based on your needs
                # frame_time = 1.0 / target_fps
                # processing_time = time.time() - current_time
                # sleep_time = max(0, frame_time - processing_time)
                # time.sleep(sleep_time)

        except Exception as e:
            log_error("CAMERA", f"Main loop error: {e}")
            import traceback
            log_error("CAMERA", traceback.format_exc())
        
        # Clean up
        try:
            if hasattr(self, 'cap') and self.cap is not None and self.running:
                self.cap.release()
                self.cap = None
        except Exception as e:
            log_error("CAMERA", f"Error cleaning up camera in tracking loop: {e}")
        # Just in case it crashes or something
        self.running = False
        if self.debug_visualization:
            cv2.destroyAllWindows()
        
        log_info("CAMERA", "Tracking loop ended")


    def process_commands(self):
        """Process incoming commands and control the camera"""
        log_info("CAMERA", "Starting command processing loop")
        self.is_first_run = True

        while True:
            try:
                # Receive command
                cmd = self.command_subscriber.recv_json(zmq.NOBLOCK)
                log_info("CAMERA", f"Received command: {cmd['command']}")

                if cmd['command'] == "SEARCH":
                    if not self.running:
                        log_info("CAMERA", "Starting tracking")     


                        # First, ensure any existing resources are fully cleaned up
                        try:
                            if hasattr(self, 'cap') and self.cap is not None:
                                log_info("CAMERA", "Releasing old camera handle")
                                self.cap.release()
                                self.cap = None
                                time.sleep(1.0)  # Give system time to fully release
                        except Exception as e:
                            log_error("CAMERA", f"Error cleaning up old camera: {e}")
                        
                        # Now try to initialize the camera
                        try:
                            # Initialize camera
                            log_info("CAMERA", "Initializing camera...")
                            self._init_camera(self.config)
                            
                            # if not self.is_first_run:
                            #     log_info("CAMERA", "Disabling visualization for non-first run")
                            #     self.debug_visualization = False
                            # else:
                            #     self.is_first_run = False
                            # Test if camera is actually working
                            ret, _ = self.cap.read()
                            if not ret:
                                raise Exception("Camera read test failed")
                            
                            log_info("CAMERA", "Camera is working, starting tracking thread")
                            self.running = True
                            self.tracking_thread = Thread(target=self._getting_tracking_data)
                            self.tracking_thread.daemon = True
                            self.tracking_thread.start()
                            log_info("CAMERA", "Tracking thread started")
                        except Exception as e:
                            log_error("CAMERA", f"Failed to start camera: {e}")
                            import traceback
                            log_error("CAMERA", traceback.format_exc())
                            self.running = False
                            # Try to release camera if it was partially initialized
                            try:
                                if hasattr(self, 'cap') and self.cap is not None:
                                    self.cap.release()
                                    self.cap = None
                            except:
                                pass

                elif cmd['command'] == "STOP":
                    if self.running:
                        log_info("CAMERA", "Stopping tracking")
                        self.running = False
                        
                        # Wait for the thread to finish processing
                        time.sleep(1.0)  # Longer wait time

                        # release camera resources
                        try:
                            if hasattr(self, 'cap') and self.cap is not None:
                                log_info("CAMERA", "Releasing camera...")
                                self.cap.release()
                                self.cap = None
                                log_info("CAMERA", "Camera released")
                        except Exception as e:
                            log_error("CAMERA", f"Error releasing camera: {e}")
            
                        # Make sure thread is terminated
                        if hasattr(self, 'tracking_thread') and self.tracking_thread is not None:
                            log_info("CAMERA", "Waiting for tracking thread to terminate...")
                            self.tracking_thread.join(timeout=3.0)
                            self.tracking_thread = None
                        
                        # Reset tracking data
                        self.distance_history = []
                        self.id_tracked = 0
                        
                        log_info("CAMERA", "Camera stopped and resources cleaned up")

            except zmq.Again:
                pass

            time.sleep(0.01)

    def cleanup(self):
        """Clean up resources before exit"""
        log_info("CAMERA", "Cleaning up resources")
        self.running = False
        self.cap.release()
        # if self.debug_visualization:
        #     cv2.destroyAllWindows()
        if  hasattr(self, 'video_publisher'):
           self.video_publisher.close()
        self.context.term()