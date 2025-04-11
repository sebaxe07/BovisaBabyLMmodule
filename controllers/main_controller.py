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
from utils.colored_logger import log_info, log_error, log_debug

class MainController:
    def __init__(self, config):
        log_info("CONTROLLER", "Initializing main controller")
        self.config = config
        self.arduino = ArduinoInterface(config['arduino'])
        self._setup_communication()
        self._setup_lidar()
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
        
        # Add camera subscription if needed
        # self.camera_sub = ...

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
            try:
                msg = self.subscriber.recv_json(zmq.NOBLOCK)
                if msg['type'] == 'obstacles':
                    self.last_obstacles = msg['data']
                    
                    if self.current_state == "TRACKING":
                        action = self._avoidance_strategy(msg['data'])
                        self.arduino.send_command(action)
                        
            except zmq.Again:
                pass
            
            # Add human detection handling here
            # ...
            
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