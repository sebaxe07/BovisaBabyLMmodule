import zmq
import time
import random
from threading import Thread
from utils.colored_logger import log_info, log_error
import yaml

# filepath: /home/sebas/BovisaBabyLMmodule/sensors/camera_client.py

class MockCameraClient:
    def __init__(self, config):
        log_info("CAMERA", "Initializing mock camera client")
        self.config = config
        self.context = zmq.Context()
        self.running = False
        
        # Default position values
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
                        self.tracking_thread = Thread(target=self._generate_tracking_data)
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
        self.context.term()