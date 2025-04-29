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
        # Camera is now on a different device, no need to set it up locally
        self.current_state = "IDLE"
        self.target_direction = None
        self.target_distance = None
        self.target_position = None
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
        self.camera_subscriber.connect("tcp://192.168.1.50:5558")
        self.camera_subscriber.setsockopt_string(zmq.SUBSCRIBE, '')
        log_info("CONTROLLER", "Connected to camera data channel on port 5558")
        
        # Create publisher for camera commands
        self.camera_command_publisher = context.socket(zmq.PUB)
        self.camera_command_publisher.bind("tcp://192.168.1.40:5557")
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

    def _avoidance_strategy(self, obstacles):
        """Simple obstacle avoidance logic"""
        front_obstacles = [o for o in obstacles if abs(o['y']) < 0.2]
        # We already know that we have obstacles in front
        # Check whats the best direction to go
        if front_obstacles:
            # Check if there are obstacles on the left and right
            left_obstacles = [o for o in front_obstacles if o['y'] > -0.2 and o['y'] < 0]
            right_obstacles = [o for o in front_obstacles if o['y'] < 0.2 and o['y'] >= 0]
            
            if len(left_obstacles) > len(right_obstacles):
                return "right"
            else:
                return "left"
            

    def process_messages(self):
        last_command_time = 0
        last_direction = None
        throttle_interval = 0.2  # seconds between commands
        tolerance = self.config['camera']['tolerance']

        not_found = False

        # Human tracking data from camera
        human_position = None
        human_distance = None

        # Avoidance routine variables
        avoidance_state = None
        avoidance_start_time = 0
        avoidance_direction = None
        turn_duration = self.config['robot']['turn_duration']  # seconds to turn
        forward_duration = self.config['robot']['forward_duration']  # seconds to move forward
        return_duration = self.config['robot']['return_duration']  # seconds to turn back
            
        while True:

            # Check for camera tracking data
            try:
                camera_msg = self.camera_subscriber.recv_json(zmq.NOBLOCK)
                current_time = time.time()
                
                # Process camera tracking data with throttling
                if self.current_state == "AVOIDING":
                    # We are in avoidance mode, let the LIDAR handle it
                    continue
                else:
                    # Process camera tracking data
                    if camera_msg['type'] == 'TRACKING':
                        not_found = False
                        self.current_state = "TRACKING"
                        self.target_position = camera_msg['x_position']
                        self.target_distance = camera_msg['distance']
                        
                        # Store human position for obstacle comparison
                        human_position = self.target_position
                        human_distance = self.target_distance
                        
                        # Convert camera coordinates to LIDAR-style coordinates
                        # camera x_position is lateral offset (-10 to +10),
                        # and distance is forward distance in meters
                        human_x_lidar = human_distance  # forward = x in LIDAR
                        human_y_lidar = human_position  # right-positive to left-positive conversion
                        #log_debug("CONTROLLER", f"Human at LIDAR coords: ({human_x_lidar:.2f}, {human_y_lidar:.2f})")


                        # Check if we are close to the target
                        if self.target_distance <= self.config['camera']['close_distance']:
                            # We are close to the target, stop arduino, change state and stop camera
                            self.arduino.send_command("found")
                            self.current_state = "IDLE"
                            self.camera_command_publisher.send_json({
                                'command': 'STOP'
                            })
                            # Clear the target direction and distance
                            self.target_position = None
                            self.target_distance = None
                            self.target_direction = None
                            last_direction = None
                            last_command_time = 0

                            log_info("CONTROLLER", "Stopping due to close target")
                            continue
                        

                        
                        # Only send command if time elapsed and direction changed
                        self.arduino.send_command(self.target_position)
                        log_info("CONTROLLER", f"Sent command: {self.target_direction }")
                
                    elif camera_msg['type'] == 'NOTFOUND' and not not_found:
                        # We lost track of the human or not found
                        not_found = True
                        self.current_state = "SEARCH"
                        self.target_position = None
                        self.target_distance = None
                        self.target_direction = None
                        last_direction = None
                        last_command_time = 0
                        
                        log_info("CONTROLLER", "Lost track, stopping movement")
                        self.arduino.send_command("stop")

            except zmq.Again:
                pass
            
            # Process LIDAR data
            try:
                msg = self.subscriber.recv_json(zmq.NOBLOCK)
                if msg['type'] == 'obstacles':
                    self.last_obstacles = msg['data']
                    
                    # Only process obstacles in TRACKING mode (not during avoidance)
                    if self.current_state == "TRACKING" and not avoidance_state:
                        if self.last_obstacles and self.target_direction == "forward":
                            # Filter out obstacles that match the human position
                            non_human_obstacles = []

                            for obstacle in self.last_obstacles:
                                # Only compare when we have human position data
                                if human_distance is not None:
                                    # Convert to common coordinate system for comparison
                                    obstacle_distance = obstacle['distance'] 
                                    
                                    # Calculate distance between obstacle and expected human position
                                    position_diff = abs(obstacle['x'] - human_x_lidar) + abs(obstacle['y'] - human_y_lidar)
                                    distance_diff = abs(obstacle_distance - human_distance)
                                    log_debug("CONTROLLER", f"Obstacle at ({obstacle['x']:.2f}, {obstacle['y']:.2f}) "
                                            f"with distance {obstacle_distance:.2f}m, human at ({human_x_lidar:.2f}, {human_y_lidar:.2f}) "
                                            f"with distance {human_distance:.2f}m - position_diff: {position_diff:.2f}, distance_diff: {distance_diff:.2f}")
                                    # If obstacle is close to where we expect the human to be
                                    if position_diff < 0.5 and distance_diff < 0.5:  # 0.5m threshold
                                        log_debug("CONTROLLER", f"Obstacle at ({obstacle['x']:.2f}, {obstacle['y']:.2f}) " 
                                                f"identified as human - ignoring for avoidance")
                                        continue
                                
                                # If we get here, it's not the human
                                non_human_obstacles.append(obstacle)
                        
                            # Check if any non-human obstacles in front
                            front_obstacles = [o for o in non_human_obstacles if abs(o['y']) < 0.2]
                            if front_obstacles:
                                log_info("CONTROLLER", "Non-human obstacle detected, starting avoidance")
                                self.current_state = "AVOIDING"
                                avoidance_direction = self._avoidance_strategy(front_obstacles)
                                avoidance_state = "AVOIDING_TURN"
                                avoidance_start_time = current_time
                                self.arduino.send_command(avoidance_direction)
                                log_info("CONTROLLER", f"Avoidance step 1: Turning {avoidance_direction}")
            except zmq.Again:
                pass

            # Process avoidance state machine
            if self.current_state == "AVOIDING" and avoidance_state:
                elapsed = current_time - avoidance_start_time
                
                if avoidance_state == "AVOIDING_TURN" and elapsed >= turn_duration:
                    # Step 2: Move forward
                    avoidance_state = "AVOIDING_FORWARD"
                    avoidance_start_time = current_time
                    self.arduino.send_command("forward")
                    log_info("CONTROLLER", "Avoidance step 2: Moving forward")
                    
                elif avoidance_state == "AVOIDING_FORWARD" and elapsed >= forward_duration:
                    # Step 3: Turn back to original direction
                    avoidance_state = "AVOIDING_RETURN"
                    avoidance_start_time = current_time
                    # Reverse the direction for return turn
                    return_direction = "left" if avoidance_direction == "right" else "right"
                    self.arduino.send_command(return_direction)
                    log_info("CONTROLLER", f"Avoidance step 3: Turning back {return_direction}")
                    
                elif avoidance_state == "AVOIDING_RETURN" and elapsed >= return_duration:
                    # Step 4: Return to tracking
                    log_info("CONTROLLER", "Avoidance complete, returning to tracking")
                    self.current_state = "TRACKING"
                    avoidance_state = None         
                    # Clear the target direction and distance so that camera can re-evaluate  
                    self.target_position = None
                    self.target_distance = None
                    self.target_direction = None
                    last_direction = None
                    last_command_time = 0   


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