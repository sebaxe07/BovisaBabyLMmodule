# controllers/main_controller.py
import sys
import os
from threading import Thread
# Add project root to Python path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import zmq
import time
from motor.arduino_interface import ArduinoInterface
from sensors.lidar_processor import LidarProcessor
from sensors.camera_client import MockCameraClient
from utils.colored_logger import log_info, log_error, log_debug

class MainController:
    def __init__(self, config):
        log_info("CONTROLLER", "Initializing main controller")
        self.config = config
        self.arduino = ArduinoInterface(config['arduino'])
        self._setup_communication()
        self._setup_lidar()
        self._setup_camera()
        self.current_state = "IDLE"
        self.target_direction = None
        self.last_obstacles = []
        log_info("CONTROLLER", "Initialization complete")

    def _setup_communication(self):
        context = zmq.Context()
        # Subscribe to LIDAR data
        self.subscriber = context.socket(zmq.SUB)
        self.subscriber.connect("tcp://localhost:5555")
        self.subscriber.setsockopt_string(zmq.SUBSCRIBE, '')
        
        # Add subscription for UI commands
        self.command_subscriber = context.socket(zmq.SUB)
        self.command_subscriber.connect("tcp://localhost:5556")
        self.command_subscriber.setsockopt_string(zmq.SUBSCRIBE, '')
        log_info("CONTROLLER", "Connected to command channel on port 5556")

        # Add subscription for camera tracking data
        self.camera_subscriber = context.socket(zmq.SUB)
        self.camera_subscriber.connect("tcp://localhost:5558")
        self.camera_subscriber.setsockopt_string(zmq.SUBSCRIBE, '')
        log_info("CONTROLLER", "Connected to camera data channel on port 5558")
        
        # Create publisher for camera commands
        self.camera_command_publisher = context.socket(zmq.PUB)
        self.camera_command_publisher.bind("tcp://*:5557")
        log_info("CONTROLLER", "Created camera command channel on port 5557")

    def _setup_lidar(self):
        """Start LIDAR processor in a separate thread"""
        log_info("CONTROLLER", "Starting LIDAR processor")
        
        def run_lidar():
            processor = LidarProcessor(self.config['lidar'])
            processor.start()
            # Store reference to allow clean shutdown later
            self.lidar_processor = processor
        
        # Start in a daemon thread so it exits when the main program exits
        self.lidar_thread = Thread(target=run_lidar)
        self.lidar_thread.daemon = True
        self.lidar_thread.start()
        
        # Give the LIDAR processor a moment to start
        time.sleep(1.0)
        log_info("CONTROLLER", "LIDAR processor started")

    def _setup_camera(self):
        """Start camera processing in a separate thread"""
        log_info("CONTROLLER", "Starting camera processor")
        def run_camera():
            camera = MockCameraClient(self.config['camera'])
            camera.process_commands()
            # Store reference to allow clean shutdown later
            self.camera_client = camera
        
        # Start in a daemon thread so it exits when the main program exits
        self.camera_thread = Thread(target=run_camera)
        self.camera_thread.daemon = True
        self.camera_thread.start()

        # Give the camera processor a moment to start
        time.sleep(1.0)
        log_info("CONTROLLER", "Camera processor started")


    def _avoidance_strategy(self, obstacles):
        """Simple obstacle avoidance logic"""
        front_obstacles = [o for o in obstacles if abs(o['y']) < 0.2]
        if not front_obstacles:
            return "forward"
        
        # Find safest direction
        right_clear = all(o['x'] < -0.3 for o in front_obstacles)
        left_clear = all(o['x'] > 0.3 for o in front_obstacles)
        
        if right_clear:
            return "right"
        elif left_clear:
            return "left"
        else:
            return "stop"

    def process_messages(self):
        """Main loop to process incoming messages"""
        log_info("CONTROLLER", "Starting message processing loop")
        while True:

            # Check for camera data
            try:
                camera_msg = self.camera_subscriber.recv_json(zmq.NOBLOCK)
                log_info("CONTROLLER", f"Received camera data: {camera_msg['type']}")
                
                # Process camera tracking data
                if camera_msg['type'] == 'TRACKING':
                    log_info("CONTROLLER", f"Human detected at distance: {camera_msg['distance']}")
                    # Change state to TRACKING
                    self.current_state = "TRACKING"
                    self.target_direction = camera_msg['x_position']
                    # Send command to Arduino based on target direction with a tolerance
                    tolerance = self.config['camera']['tolerance']  # Get tolerance from config
                    if self.target_direction > tolerance:
                        self.arduino.send_command("right")
                    elif self.target_direction < -tolerance:
                        self.arduino.send_command("left")
                    else:
                        self.arduino.send_command("forward")
            except zmq.Again:
                pass

            # # Check for LIDAR data
            # try:
            #     msg = self.subscriber.recv_json(zmq.NOBLOCK)
            #     if msg['type'] == 'obstacles':
            #         self.last_obstacles = msg['data']
            #         if self.current_state == "TRACKING":
            #             action = self._avoidance_strategy(msg['data'])
            #             self.arduino.send_command(action)
            # except zmq.Again:
            #     pass
            
            # Check for UI commands
            try:
                cmd = self.command_subscriber.recv_json(zmq.NOBLOCK)
                log_info("CONTROLLER", f"Received command: {cmd['command']}")
                
                if cmd['command'] == 'set_state':
                    self.current_state = cmd['state']
                    log_info("CONTROLLER", f"State changed to: {self.current_state}")
                    # If command is search, send search command to camera
                    if cmd['state'] == "SEARCH":
                        self.camera_command_publisher.send_json({
                            'command': 'SEARCH'
                        })
                        log_info("CONTROLLER", "Sent search command to camera")
                    
                    elif cmd['state'] == 'IDLE':
                        self.camera_command_publisher.send_json({
                            'command': 'STOP'
                        })
                        log_info("CONTROLLER", "Sent stop command to camera")

                elif cmd['command'] == 'UPDATE_HUMAN_POSITION':
                    # Send updated position to camera
                    self.camera_command_publisher.send_json({
                        'command': 'UPDATE_HUMAN_POSITION',
                        'x_position': cmd['x_position'],
                        'distance': cmd['distance']
                    })
                    log_info("CONTROLLER", f"Updated human position: x={cmd['x_position']}, distance={cmd['distance']}")

                    
                elif cmd['command'] == 'motor':
                    self.arduino.send_command(cmd['action'])
                    log_info("CONTROLLER", f"Sent motor command: {cmd['action']}")
                    
                    
            except zmq.Again:
                pass
            
            time.sleep(0.01)

    def cleanup(self):
        """Clean up resources before exit"""
        log_info("CONTROLLER", "Cleaning up resources")
        if hasattr(self, 'lidar_processor'):
            try:
                self.lidar_processor.stop()
                log_info("CONTROLLER", "LIDAR processor stopped")
            except Exception as e:
                log_error("CONTROLLER", f"Error stopping LIDAR: {e}")
        
        if hasattr(self, 'arduino'):
            try:
                self.arduino.send_command("stop")
                self.arduino.cleanup()
                log_info("CONTROLLER", "Arduino stopped")
            except Exception as e:
                log_error("CONTROLLER", f"Error stopping Arduino: {e}")

if __name__ == "__main__":
    import yaml
    
    # Load the config
    try:
        with open('../config/settings.yaml', 'r') as f:
            config = yaml.safe_load(f)
    except Exception as e:
        log_error("CONTROLLER", f"Error loading config: {e}")
        sys.exit(1)
    
    # Create and run the controller
    controller = None
    try:
        controller = MainController(config)
        controller.process_messages()
    except KeyboardInterrupt:
        log_info("CONTROLLER", "Keyboard interrupt received, shutting down...")
    except Exception as e:
        log_error("CONTROLLER", f"Unexpected error: {e}")
    finally:
        if controller:
            controller.cleanup()