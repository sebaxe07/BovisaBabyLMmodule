# controllers/main_controller.py
# This module contains the main controller for the robot system.
# It handles tracking of humans, obstacle avoidance, and GPS geofencing.
# 
# LIDAR Avoidance System:
# The LIDAR avoidance system uses analog values (-10 to 10) similar to camera tracking
# to provide smooth, continuous avoidance movements. Instead of a discrete state machine
# with set turn durations, the system dynamically adjusts steering based on obstacle
# positions and distances. This allows the robot to make smaller adjustments when
# obstacles are not directly in front.
#
import sys
import os
import math
from threading import Thread
# Add project root to Python path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import zmq
import time
from motor.arduino_interface import ArduinoInterface
from sensors.lidar_processor import LidarProcessor
from sensors.gps_processor import GpsProcessor
from utils.colored_logger import log_info, log_error, log_debug

class MainController:
    def __init__(self, config):
        log_info("CONTROLLER", "Initializing main controller")
        self.config = config
        self.arduino = ArduinoInterface(config['arduino'])
        self._setup_communication()
        self._setup_lidar()
        self._setup_gps()
        # Camera is now on a different device, no need to set it up locally
        self.current_state = "IDLE"
        self.target_direction = None
        self.target_distance = None
        self.target_position = None
        self.original_tracking_position = None  # For storing tracking position during avoidance
        self.last_obstacles = []
        
        # Geofence status
        self.inside_geofence = True
        self.returning_to_geofence = False
        self.return_bearing = 0  # Direction to geofence center
        
        log_info("CONTROLLER", "Initialization complete")

    def _setup_communication(self):
        context = zmq.Context()
        # Subscribe to LIDAR data
        self.subscriber = context.socket(zmq.SUB)
        self.subscriber.connect(f"tcp://localhost:{self.config['lidar']['communication']['port']}")
        self.subscriber.setsockopt_string(zmq.SUBSCRIBE, '')
        
        # Add subscription for UI commands
        self.command_subscriber = context.socket(zmq.SUB)
        self.command_subscriber.connect(f"tcp://localhost:{self.config['controller']['communication']['command_port']}")
        self.command_subscriber.setsockopt_string(zmq.SUBSCRIBE, '')
        log_info("CONTROLLER", f"Connected to command channel on port {self.config['controller']['communication']['command_port']}")

        # Add subscription for camera tracking data
        self.camera_subscriber = context.socket(zmq.SUB)
        self.camera_subscriber.connect(f"tcp://{self.config['camera']['communication']['ip']}:{self.config['camera']['communication']['tracking_port']}")
        self.camera_subscriber.setsockopt_string(zmq.SUBSCRIBE, '')
        log_info("CONTROLLER", f"Connected to camera data channel on port {self.config['camera']['communication']['tracking_port']}")
        
        # Add subscription for GPS data
        self.gps_subscriber = context.socket(zmq.SUB)
        self.gps_subscriber.connect(f"tcp://localhost:{self.config['gps']['communication']['port']}")
        self.gps_subscriber.setsockopt_string(zmq.SUBSCRIBE, '')
        log_info("CONTROLLER", f"Connected to GPS data channel on port {self.config['gps']['communication']['port']}")
        
        # Create publisher for camera commands
        self.camera_command_publisher = context.socket(zmq.PUB)
        self.camera_command_publisher.bind(f"tcp://{self.config['controller']['communication']['ip']}:{self.config['camera']['communication']['command_port']}")
        log_info("CONTROLLER", f"Created camera command channel on address tcp://{self.config['controller']['communication']['ip']}:{self.config['camera']['communication']['command_port']}")

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

    def _setup_gps(self):
        """Start GPS processor in a separate thread"""
        log_info("CONTROLLER", "Starting GPS processor")
        
        def run_gps():
            processor = GpsProcessor(self.config['gps'])
            processor.start()
            # Store reference to allow clean shutdown later
            self.gps_processor = processor
        
        # Start in a daemon thread so it exits when the main program exits
        self.gps_thread = Thread(target=run_gps)
        self.gps_thread.daemon = True
        self.gps_thread.start()
        
        # Give the GPS processor a moment to start
        time.sleep(1.0)
        log_info("CONTROLLER", "GPS processor started")

    def _avoidance_strategy(self, obstacles):
        """
        Advanced obstacle avoidance using analog position values (-10 to 10)
        to provide smooth, intelligent turning behavior.
        
        The algorithm works as follows:
        1. Direction determination: 
           - If obstacle is on the left side, turn right (negative value)
           - If obstacle is on the right side, turn left (positive value)
        
        2. Urgency calculation:
           - Center obstacles generate higher urgency (sharper turns)
           - Edge obstacles generate lower urgency (gentler turns)
           - Closer obstacles increase urgency
        
        3. Emergency stop:
           - Very close obstacles directly in front trigger emergency stop
        
        The final value is direction × urgency × max_turn_value (10)
        """
        # Get the front detection cone angle from config, default to 60 degrees if not specified
        front_cone_angle = self.config['lidar']['front_detection_angle']
        
        # Filter for obstacles in the front cone based on angle
        front_obstacles = []
        for o in obstacles:
            if o['x'] > 0:  # Only consider obstacles in front (positive x)
                # Calculate angle in degrees from forward direction
                obstacle_angle = math.degrees(math.atan2(o['y'], o['x']))
                # Check if within our cone angle (e.g., ±30 degrees)
                if abs(obstacle_angle) <= (front_cone_angle / 2):
                    front_obstacles.append(o)
        
        if not front_obstacles:
            # No obstacles in front
            return 0.0  # Go straight
        
        # Find the closest obstacle in the cone
        closest_obstacle = min(front_obstacles, key=lambda o: o['distance'])
        
        # Calculate the angle of the obstacle from the center
        obstacle_angle = math.degrees(math.atan2(closest_obstacle['y'], closest_obstacle['x']))
        
        # Check if we're in an emergency stop situation (obstacle very close and near center)
        emergency_stop_distance = self.config['lidar']['emergency_stop_distance']
        emergency_stop_angle = self.config['lidar']['emergency_stop_angle']

        if closest_obstacle['distance'] < emergency_stop_distance and abs(obstacle_angle) < emergency_stop_angle:
            log_error("CONTROLLER", f"EMERGENCY STOP! Obstacle too close ({closest_obstacle['distance']:.2f}m) and centered (angle: {obstacle_angle:.1f}°)")
            return None  # Special return value to indicate emergency stop
        
        # CORRECTED AVOIDANCE LOGIC:
        # 1. Base direction: determine which way to turn based on obstacle position
        # If obstacle is on the right (negative angle), turn left (positive value)
        # If obstacle is on the left (positive angle), turn right (negative value)
        turn_direction = -1.0 if obstacle_angle > 0 else 1.0
        
        # 2. DIRECT ANGLE MAPPING: Convert obstacle angle directly to a turn magnitude
        # Map from angle range [-max_angle, max_angle] to turn magnitude [10, 3]
        # Center angle (0°) = maximum turn value (10)
        # Edge angles (±max_angle) = minimum turn value (min_turn_value)
        max_detection_angle = front_cone_angle / 2
        min_turn_value = 3.0  # Minimum turn value at the edges of detection cone
        
        # Linear mapping from angle to turn magnitude
        # As |angle| approaches 0 (center), turn_magnitude approaches 10
        # As |angle| approaches max_angle, turn_magnitude approaches min_turn_value
        angle_ratio = abs(obstacle_angle) / max_detection_angle  # 0 at center, 1 at edges
        turn_magnitude = 10.0 - (angle_ratio * (10.0 - min_turn_value))
        
        # 3. Distance urgency: Higher when obstacle is closer
        # This will boost the turn magnitude for closer obstacles
        distance_factor = 1.0 - (closest_obstacle['distance'] / self.config['lidar']['safety_distance'])
        distance_factor = max(0.5, min(1.0, distance_factor))  # Keep in range [0.5, 1.0]
        
        # 4. Apply distance factor to increase turn magnitude for closer obstacles
        # This ensures we still get full 10 for center obstacles, but boosted turn for close ones
        final_turn_magnitude = turn_magnitude * (1.0 + (distance_factor - 0.5) * 0.5)
        final_turn_magnitude = min(10.0, final_turn_magnitude)  # Cap at 10
        
        # 5. Calculate final avoidance value with sign from turn_direction
        avoidance_value = turn_direction * final_turn_magnitude
        
        # Constrain to -10 to 10 range
        avoidance_value = max(-10.0, min(10.0, avoidance_value))
        
        log_debug("CONTROLLER", f"Avoidance value: {avoidance_value:.2f}, " 
                 f"obstacle at ({closest_obstacle['x']:.2f}, {closest_obstacle['y']:.2f}), "
                 f"angle: {obstacle_angle:.1f}°, distance: {closest_obstacle['distance']:.2f}m, "
                 f"turn_direction: {turn_direction}, angle_ratio: {angle_ratio:.2f}, "
                 f"turn_magnitude: {turn_magnitude:.2f}, distance_factor: {distance_factor:.2f}, "
                 f"final_magnitude: {final_turn_magnitude:.2f}, cone: ±{front_cone_angle/2:.1f}°")
        
        return avoidance_value

    def _process_geofence_alert(self, alert):
        """Process a geofence alert from the GPS"""
        alert_level = alert.get('alert_level')
        direction = alert.get('direction_to_center')
        
        if alert_level == 'critical':
            log_error("CONTROLLER", "CRITICAL GEOFENCE ALERT - Outside permitted area!")
            distance_outside = alert.get('distance_outside', 0)
            log_error("CONTROLLER", f"Robot is {distance_outside:.2f}m outside permitted area")
            log_error("CONTROLLER", f"Initiating return to geofence, bearing {direction:.1f}°")
            
            # Override current state - geofence takes priority
            self.previous_state = self.current_state  # Store current state to restore later
            self.current_state = "RETURNING"
            self.returning_to_geofence = True
            self.return_bearing = direction
            
            # Stop camera tracking if active
            self.camera_command_publisher.send_json({'command': 'STOP'})
            
            # Convert bearing to robot command
            # Simple conversion - could be made more sophisticated
            if 45 <= direction < 135:  # Roughly East
                self.arduino.send_command("right")
            elif 225 <= direction < 315:  # Roughly West
                self.arduino.send_command("left")
            else:  # Roughly North or South - go forward
                self.arduino.send_command("forward")
            
        elif alert_level == 'warning':
            log_info("CONTROLLER", "GEOFENCE WARNING - Approaching boundary")
            distance_edge = alert.get('distance_to_edge', 0)
            log_info("CONTROLLER", f"Robot is {distance_edge:.2f}m from boundary")
            
            # Don't override state but log warning
            if self.current_state == "TRACKING":
                log_info("CONTROLLER", "Continuing tracking but monitoring geofence")
            
            # We could slow down or take other precautionary measures here

    def _bearing_to_robot_command(self, bearing, current_position=None):
        """Convert a compass bearing to a robot command"""
        # This is a simplified version - could be improved with actual heading data
        
        # Divide the compass into quadrants
        if 315 <= bearing or bearing < 45:  # North
            return "forward"
        elif 45 <= bearing < 135:  # East
            return "right"
        elif 135 <= bearing < 225:  # South
            return "reverse"
        elif 225 <= bearing < 315:  # West
            return "left"
            
        # Default if bearing calculation fails
        return "forward"

    def _process_gps_data(self, message):
        """Process GPS data message"""
        # Store GPS data in controller
        self.current_gps = message
        
        # Update dashboard with GPS status
        if hasattr(self, 'gps_processor'):
            status_info = self.gps_processor.get_status_info()
            interpretation = self.gps_processor.interpret_gps_status()
            
            # Log detailed interpretations when status changes or periodically
            if (not hasattr(self, '_last_gps_status') or 
                self._last_gps_status != status_info['status_message'] or
                time.time() - getattr(self, '_last_gps_log_time', 0) > 30):
                
                log_info("Controller", f"GPS Status: {interpretation}")
                self._last_gps_status = status_info['status_message']
                self._last_gps_log_time = time.time()
                
                # If no fix, show more detailed diagnostics
                if not status_info['has_fix']:
                    log_debug("Controller", f"GPS Diagnostics: {status_info['satellites']} satellites, "
                             f"HDOP: {status_info['hdop']:.1f}, Fix quality: {status_info['fix_quality']}")
        
        # Handle geofence alerts if enabled
        if 'geofence' in message and self.config['geofence']['enabled']:
            if not message['geofence']['inside']:
                log_error("Controller", f"GEOFENCE ALERT: Outside geofence by "
                         f"{-message['geofence']['distance_to_edge']:.1f}m")
                # Take action based on geofence alert

    def process_messages(self):
        last_command_time = 0
        last_direction = None
        throttle_interval = 0.2  # seconds between commands
        tolerance = self.config['camera']['tolerance']

        not_found = False

        # Human tracking data from camera
        human_position = None
        human_distance = None

        # Dynamic avoidance variables
        avoidance_start_time = 0
        avoidance_direction = 0.0
        # Get avoidance timeout from config or use default
        avoidance_timeout = self.config['robot']['avoidance_timeout']
            
        while True:
            current_time = time.time()
            
            # Check for GPS data (high priority)
            try:
                gps_msg = self.gps_subscriber.recv_json(zmq.NOBLOCK)
                
                if gps_msg['type'] == 'gps_data':
                    # Update local position information
                    if 'geofence' in gps_msg:
                        self.inside_geofence = gps_msg['geofence']['inside']
                        
                    # If we were returning to geofence and now inside, restore previous state
                    if self.returning_to_geofence and self.inside_geofence:
                        log_info("CONTROLLER", "Successfully returned to geofence area")
                        self.returning_to_geofence = False
                        self.current_state = getattr(self, 'previous_state', "IDLE")
                        log_info("CONTROLLER", f"Restoring previous state: {self.current_state}")
                        
                        # Stop the robot to reassess
                        self.arduino.send_command("stop")
                        
                        # If we were tracking, resume camera operations
                        if self.current_state == "TRACKING":
                            self.camera_command_publisher.send_json({'command': 'SEARCH'})
                            
                elif gps_msg['type'] == 'geofence_alert':
                    # Process geofence alerts
                    self._process_geofence_alert(gps_msg)
            except zmq.Again:
                pass

            # If returning to geofence, this takes precedence over other behaviors
            if self.returning_to_geofence:
                # Check if we need to update our heading based on new bearing data
                if current_time - last_command_time > throttle_interval:
                    command = self._bearing_to_robot_command(self.return_bearing)
                    self.arduino.send_command(command)
                    last_command_time = current_time
                    
                # Skip other processing while returning to geofence
                time.sleep(0.01)
                continue

            # Check for camera tracking data
            try:
                camera_msg = self.camera_subscriber.recv_json(zmq.NOBLOCK)
                
                # Process camera tracking data with throttling
                if self.current_state == "AVOIDING":
                    # Store the tracking data but don't act on it immediately
                    # Let the avoidance system handle movement until avoidance is complete
                    if camera_msg['type'] == 'TRACKING':
                        # Store human position for obstacle comparison and future tracking
                        self.target_position = camera_msg['x_position']
                        self.target_distance = camera_msg['distance']
                        human_position = self.target_position
                        human_distance = self.target_distance
                        
                        # Convert camera coordinates to LIDAR-style coordinates
                        human_x_lidar = human_distance  # forward = x in LIDAR
                        human_y_lidar = human_position  # right-positive to left-positive conversion
                        
                        # Store this as the original tracking position for when avoidance completes
                        self.original_tracking_position = self.target_position
                        
                        log_debug("CONTROLLER", f"Human tracked during avoidance at position: {self.target_position:.2f}")
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
                        log_info("CONTROLLER", f"Sent command: {self.target_position}")
                
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
                    
                    # Process obstacles in both TRACKING and AVOIDING modes
                    if (self.current_state == "TRACKING" or self.current_state == "AVOIDING") and not self.returning_to_geofence:
                        if self.last_obstacles:
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
                                    
                                    # # Debug logs only when not in avoidance mode to reduce log spam
                                    # if self.current_state != "AVOIDING":
                                    #     log_debug("CONTROLLER", f"Obstacle at ({obstacle['x']:.2f}, {obstacle['y']:.2f}) "
                                    #             f"with distance {obstacle_distance:.2f}m vs human at ({human_x_lidar:.2f}, {human_y_lidar:.2f}) - "
                                    #             f"diff: {position_diff:.2f}, {distance_diff:.2f}")
                                    
                                    # If obstacle is close to where we expect the human to be (slightly increased threshold)
                                    human_threshold = 0.6  # Slightly increased from 0.5 for more reliable detection
                                    if position_diff < human_threshold and distance_diff < human_threshold:
                                        log_debug("CONTROLLER", f"Obstacle at ({obstacle['x']:.2f}, {obstacle['y']:.2f}) " 
                                                f"identified as human - ignoring for avoidance")
                                        continue
                                
                                # If we get here, it's not the human
                                non_human_obstacles.append(obstacle)
                            
                            # Get an analog avoidance value (-10 to 10)
                            # Get the front detection cone angle from config, default to 60 degrees
                            front_cone_angle = self.config['lidar']['front_detection_angle']
                            # Filter obstacles in front based on angle
                            front_obstacles = []
                            for o in non_human_obstacles:
                                if o['x'] > 0:  # Only consider obstacles in front (positive x)
                                    # Calculate angle in degrees from forward direction
                                    obstacle_angle = math.degrees(math.atan2(o['y'], o['x']))
                                    # Check if within our cone angle
                                    if abs(obstacle_angle) <= (front_cone_angle / 2):
                                        front_obstacles.append(o)
                                        if self.current_state != "AVOIDING":  # Reduce log spam during active avoidance
                                            log_debug("CONTROLLER", f"Obstacle in front cone: angle={obstacle_angle:.1f}°, "
                                                     f"position=({o['x']:.2f}, {o['y']:.2f}), distance={o['distance']:.2f}m")
                            
                            if front_obstacles:
                                # We have obstacles in front - calculate avoidance value
                                avoidance_value = self._avoidance_strategy(non_human_obstacles)
                                
                                # Check if this is an emergency stop situation (avoidance_value is None)
                                if avoidance_value is None:
                                    # Emergency stop - obstacle too close and centered
                                    log_error("CONTROLLER", "EMERGENCY STOP initiated - obstacle too close and centered")
                                    self.arduino.send_command("stop")
                                    
                                    # Stop the camera tracking as well
                                    self.camera_command_publisher.send_json({
                                        'command': 'STOP'
                                    })
                                    
                                    # Set state to IDLE to pause all movement
                                    self.current_state = "IDLE"
                                    log_error("CONTROLLER", "Setting state to IDLE due to emergency stop")
                                    
                                    # Reset tracking variables
                                    self.target_position = None
                                    self.target_distance = None
                                    self.target_direction = None
                                    last_direction = None
                                    last_command_time = 0
                                    
                                    # Clear the avoidance time tracking
                                    avoidance_start_time = 0
                                    
                                else:
                                    # Normal avoidance behavior
                                    if self.current_state == "TRACKING":
                                        log_error("CONTROLLER", "Obstacle detected, entering dynamic avoidance mode")
                                        self.current_state = "AVOIDING"
                                        # Store the original tracking position to restore later
                                        self.original_tracking_position = self.target_position
                                    
                                    # Send analog command directly to arduino
                                    self.arduino.send_command(float(avoidance_value))
                                    
                                    # Only log every few avoidance commands to reduce spam
                                    if current_time - last_command_time > 0.5:  # Log at most every 0.5 seconds
                                        log_error("CONTROLLER", f"Avoidance steering: {avoidance_value:.2f}")
                                        last_command_time = current_time
                                    
                                    # Set timestamp for avoidance timeout
                                    avoidance_start_time = current_time
                            
                            elif self.current_state == "AVOIDING":
                                # No more obstacles - return to tracking or go back to original heading
                                log_info("CONTROLLER", "Obstacle cleared, returning to tracking")
                                self.current_state = "TRACKING"
                                
                                # # If we have a stored tracking position, return to it
                                # if hasattr(self, 'original_tracking_position') and self.original_tracking_position is not None:
                                #     self.arduino.send_command(float(self.original_tracking_position))
                                #     log_info("CONTROLLER", f"Returning to original heading: {self.original_tracking_position}")
                                # else:
                                #     # No stored position, just go forward
                                #     self.arduino.send_command(0.0)  # Center/forward
            except zmq.Again:
                pass

            # The old step-by-step avoidance state machine has been replaced
            # with a continuous, dynamic avoidance system that uses the analog position values


            # Dynamic avoidance timeout - if we've been avoiding too long, force a return to tracking
            if self.current_state == "AVOIDING":
                avoidance_timeout = self.config['robot']['avoidance_timeout']  # Default 5 seconds
                
                if current_time - avoidance_start_time > avoidance_timeout:
                    log_info("CONTROLLER", f"Avoidance timeout after {avoidance_timeout:.1f}s, returning to tracking")
                    self.current_state = "TRACKING"
                    
                    # If we have a stored tracking position, return to it
                    if hasattr(self, 'original_tracking_position') and self.original_tracking_position is not None:
                        self.arduino.send_command(float(self.original_tracking_position))
                    else:
                        # No stored position, just go forward
                        self.arduino.send_command(0.0)  # Center/forward

            # Check for UI commands
            try:
                cmd = self.command_subscriber.recv_json(zmq.NOBLOCK)
                log_info("CONTROLLER", f"Received command: {cmd['command']}")
                
                if cmd['command'] == 'set_state':
                    # Don't allow state changes if we're returning to geofence
                    if self.returning_to_geofence:
                        log_info("CONTROLLER", "Ignoring state change request while returning to geofence")
                        continue
                        
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
                        # Reset all tracking and avoidance variables
                        self.target_position = None
                        self.target_distance = None
                        self.target_direction = None
                        last_direction = None
                        last_command_time = 0
                        # Also clear any stored original tracking position from avoidance
                        if hasattr(self, 'original_tracking_position'):
                            self.original_tracking_position = None  
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
                    # Don't allow direct motor control if we're returning to geofence
                    if self.returning_to_geofence:
                        log_info("CONTROLLER", "Ignoring motor command while returning to geofence")
                        continue
                        
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
                
        if hasattr(self, 'gps_processor'):
            try:
                self.gps_processor.stop()
                log_info("CONTROLLER", "GPS processor stopped")
            except Exception as e:
                log_error("CONTROLLER", f"Error stopping GPS: {e}")
        
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