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

        # Load camera parameters
        self.model = YOLO(config['model'])
        self.tracker = DeepSort(max_age=config['max_age'])
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
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

    def _getting_tracking_data(self):
        """Getting tracking data from the camera"""
        log_info("CAMERA", "Starting tracking loop")
        self.id_tracked = 0
        while self.running:


            # Read frame from camera
            ret, frame = self.cap.read()
            if not ret:
                log_error("CAMERA", "Failed to read frame from camera")
                break

            # YOLO Detection
            results = self.model(frame, classes=[0], conf=self.confidence_threshold)
            detections = []

            if len(results[0].boxes) > 0:
                for box in results[0].boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    h = y2 - y1
                    detections.append(([x1, y1, x2-x1, h], float(box.conf[0]), "person"))

                # DeepSORT Tracking
                tracks = self.tracker.update_tracks(detections, frame=frame)

                min_distance = 10000
                min_ltrb = [1000] * 4
                min_track_id = 1000

                for track in tracks:
                    if not track.is_confirmed():
                        continue

                    try:
                        # Ensure track_id is treated as integer
                        track_id = int(track.track_id)
                        ltrb = track.to_ltrb()

                        # Get unique color for this track
                        color = self.colors[track_id % len(self.colors)]

                        # Calculate distance
                        box_height = ltrb[3] - ltrb[1]
                        distance = self.estimate_distance(box_height)

                        # Check if we are tracking a person
                        if self.id_tracked == 0:
                            # We are not tracking a person, so we need to find one
                            if min_distance > distance:
                                min_distance = distance
                                min_ltrb = copy.copy(ltrb)
                                min_track_id = track_id
                        else:
                            # We are already tracking a person, so we need to check if it's the same one
                            if track_id == self.id_tracked:
                                #  update tracking
                                min_distance = distance
                                min_ltrb = copy.copy(ltrb)
                                min_track_id = track_id

                        log_debug("CAMERA", f"Track ID: {track_id}, Distance: {distance:.2f}m, LTRB: {ltrb}")
                        # Draw bounding box and label
                        cv2.rectangle(frame,
                                    (int(ltrb[0]), int(ltrb[1])),
                                    (int(ltrb[2]), int(ltrb[3])),
                                    color, 2)

                        cv2.putText(frame,
                                f"Person {track_id} ({distance:.2f}m)",
                                (int(ltrb[0]), int(ltrb[1])-10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                    except Exception as e:
                        log_error("CAMERA", f"Tracking error: {e}")
                        message = {
                            "type": "NOTFOUND",
                            "x_position": 0,
                            "distance": 0,
                            "human_id": 0
                        }
                        # We asume we lost the track
                        self.id_tracked = 0
                        continue


                # If no tracks are found 
                if min_distance == 10000:
                    log_error("CAMERA", "No tracks found or track lost")
                    message = {
                        "type": "NOTFOUND",
                        "x_position": 0,
                        "distance": 0,
                        "human_id": 0
                    }
                    # We asume we lost the track
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
                # We asume we lost the track
                self.id_tracked = 0

            cv2.imshow("Human Tracking", frame)
            if cv2.waitKey(1) == ord('q'):
                break
            # Send message
            self.publisher.send_json(message)
            log_info("CAMERA", f"Sent tracking data: x={message['x_position']}, dist={message['distance']}, human_id={message['human_id']}")

            # Wait before sending the next message
            time.sleep(0.5)

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
        cv2.destroyAllWindows()
        self.context.term()