# sensors/gps_processor.py
import time
import random
import math
import zmq
import serial
import threading
import pynmea2
from threading import Thread, Event
from utils.colored_logger import log_info, log_error, log_debug

class GpsProcessor:
    def __init__(self, config):
        """
        Initialize the GPS processor.
        
        Args:
            config: Configuration dictionary loaded from settings.yaml
        """
        log_info("GPS", "Initializing GPS processor")
        self.config = config
        self.mock_mode = config['mock_mode']
        self._running = Event()
        self._thread = None
        
        # Current position and status
        self.latitude = None
        self.longitude = None
        self.altitude = None
        self.speed = 0.0  # Speed in km/h
        self.course = 0.0  # Course in degrees (0-360)
        self.satellites = 0  # Number of satellites in view
        self.fix_quality = 0  # 0 = no fix, 1 = GPS fix, 2 = DGPS fix
        self.hdop = 99.9  # Horizontal dilution of precision (lower is better)
        self.last_update = 0  # Timestamp of last update
        
        # Geofence status
        self.geofence_enabled = config['geofence']['enabled']
        self.geofence_center = (config['geofence']['center_lat'], 
                                config['geofence']['center_lon'])
        self.geofence_radius = config['geofence']['radius']
        self.warning_distance = config['geofence']['warning_distance']
        self.inside_geofence = True
        self.distance_to_center = 0.0
        self.distance_to_edge = 0.0
        
        # Set up communication
        self._setup_communication()
        
        # If in mock mode, initialize the mock GPS
        if self.mock_mode:
            self._init_mock_gps()
            log_info("GPS", "Initialized in MOCK mode")
        else:
            log_info("GPS", f"Real GPS will use port {config['port']} at {config['baud_rate']} baud")

    def _init_mock_gps(self):
        """Initialize mock GPS with values from configuration"""
        self.latitude = self.config['mock_settings']['start_lat']
        self.longitude = self.config['mock_settings']['start_lon']
        self.altitude = 100.0  # Default altitude in meters
        self.fix_quality = 1  # Mock GPS has a fix by default
        self.satellites = 8  # Default number of satellites
        self.hdop = 1.2  # Default HDOP - good precision
        self.inside_geofence = True  # Start inside the geofence
        self.last_update = time.time()

    def _setup_communication(self):
        """Set up ZMQ communication sockets"""
        self.context = zmq.Context()
        self.publisher = self.context.socket(zmq.PUB)
        self.publisher.bind(f"tcp://*:{self.config['communication']['port']}")
        log_info("GPS", f"Publisher initialized on port {self.config['communication']['port']}")

    def start(self):
        """Start the GPS processing thread"""
        if self._running.is_set():
            log_info("GPS", "GPS processor is already running")
            return
            
        self._running.set()
        if self.mock_mode:
            self._thread = Thread(target=self._mock_gps_loop)
        else:
            self._thread = Thread(target=self._real_gps_loop)
            
        self._thread.daemon = True
        self._thread.start()
        log_info("GPS", "GPS processor started")

    def stop(self):
        """Stop the GPS processing thread"""
        if not self._running.is_set():
            log_info("GPS", "GPS processor is not running")
            return
            
        log_info("GPS", "Stopping GPS processor")
        self._running.clear()
        if self._thread:
            self._thread.join(timeout=2.0)
        log_info("GPS", "GPS processor stopped")

    def _mock_gps_loop(self):
        """Generate simulated GPS data"""
        log_info("GPS", "Starting mock GPS loop")
        update_interval = self.config['update_rate']
        max_movement = self.config['mock_settings']['max_movement']
        
        while self._running.is_set():
            try:
                # Simulate GPS movement with random walk
                self.latitude += random.uniform(-max_movement, max_movement)
                self.longitude += random.uniform(-max_movement, max_movement)
                
                # Update altitude, speed, and course
                self.altitude = 100.0 + random.uniform(-1.0, 1.0)
                self.speed = random.uniform(0.0, 5.0)  # 0-5 km/h
                self.course = random.uniform(0.0, 359.9)
                
                # Occasionally simulate GPS issues if enabled
                if (self.config['mock_settings']['simulate_issues'] and 
                    random.random() < self.config['mock_settings']['issue_probability']):
                    self.fix_quality = 0
                    self.satellites = random.randint(0, 3)
                    self.hdop = random.uniform(5.0, 20.0)
                    log_debug("GPS", "Simulating GPS signal issue")
                else:
                    self.fix_quality = 1
                    self.satellites = random.randint(6, 12)
                    self.hdop = random.uniform(0.8, 2.5)
                
                # Calculate geofence status
                self._update_geofence_status()
                
                # Publish the GPS data
                self._publish_gps_data()
                
                # Wait for next update
                time.sleep(update_interval)
                
            except Exception as e:
                log_error("GPS", f"Error in mock GPS loop: {e}")
                time.sleep(1)

    def _real_gps_loop(self):
        """Process real GPS data from the M8N module"""
        log_info("GPS", "Starting real GPS loop")
        
        while self._running.is_set():
            try:
                # Open serial port
                with serial.Serial(self.config['port'], self.config['baud_rate'], timeout=1) as ser:
                    log_info("GPS", f"Connected to GPS on {self.config['port']}")
                    
                    while self._running.is_set():
                        # Read a line from the GPS
                        line = ser.readline().decode('ascii', errors='replace').strip()
                        
                        # Skip empty lines
                        if not line:
                            continue
                            
                        # Try to parse the NMEA sentence
                        try:
                            msg = pynmea2.parse(line)
                            
                            # Process different types of NMEA sentences
                            if isinstance(msg, pynmea2.GGA):
                                # GGA message contains position, altitude, and fix quality
                                if msg.latitude and msg.longitude:
                                    self.latitude = msg.latitude
                                    self.longitude = msg.longitude
                                    self.altitude = float(msg.altitude) if msg.altitude else None
                                    self.fix_quality = int(msg.gps_qual)
                                    self.satellites = int(msg.num_sats) if msg.num_sats else 0
                                    self.hdop = float(msg.horizontal_dil) if msg.horizontal_dil else 99.9
                                    self.last_update = time.time()
                                    
                                    # Calculate geofence status
                                    self._update_geofence_status()
                                    
                                    # Publish the GPS data
                                    self._publish_gps_data()
                                    
                            elif isinstance(msg, pynmea2.VTG):
                                # VTG message contains speed and course
                                self.speed = float(msg.spd_over_grnd_kmph) if msg.spd_over_grnd_kmph else 0.0
                                self.course = float(msg.true_track) if msg.true_track else 0.0
                                
                        except pynmea2.ParseError:
                            # Ignore parse errors - some sentences might be corrupt
                            pass
                            
            except serial.SerialException as e:
                log_error("GPS", f"Serial error: {e}")
                # Wait before trying to reconnect
                time.sleep(5)
                
            except Exception as e:
                log_error("GPS", f"Error in GPS loop: {e}")
                time.sleep(1)

    def _update_geofence_status(self):
        """Update geofence status based on current position"""
        if not self.geofence_enabled or not self.latitude or not self.longitude:
            return
            
        # Calculate distance to geofence center
        self.distance_to_center = self._haversine_distance(
            self.latitude, self.longitude,
            self.geofence_center[0], self.geofence_center[1]
        )
        
        # Calculate distance to geofence edge
        self.distance_to_edge = max(0, self.geofence_radius - self.distance_to_center)
        
        # Check if inside geofence
        was_inside = self.inside_geofence
        self.inside_geofence = self.distance_to_center <= self.geofence_radius
        
        # Log geofence status changes
        if was_inside != self.inside_geofence:
            if self.inside_geofence:
                log_info("GPS", "Entered geofence area")
            else:
                log_error("GPS", f"LEFT GEOFENCE AREA! Distance to edge: {self.distance_to_edge:.2f}m")
        elif not self.inside_geofence:
            log_error("GPS", f"Outside geofence by {-self.distance_to_edge:.2f}m")
        elif self.distance_to_edge < self.warning_distance:
            log_info("GPS", f"Near geofence boundary! {self.distance_to_edge:.2f}m to edge")

    def _publish_gps_data(self):
        """Publish GPS data and geofence status via ZMQ"""
        # Create GPS data message
        gps_data = {
            'type': 'gps_data',
            'latitude': self.latitude,
            'longitude': self.longitude,
            'altitude': self.altitude,
            'speed': self.speed,
            'course': self.course,
            'satellites': self.satellites,
            'fix_quality': self.fix_quality,
            'hdop': self.hdop,
            'timestamp': time.time(),
            'geofence': {
                'inside': self.inside_geofence,
                'distance_to_center': self.distance_to_center,
                'distance_to_edge': self.distance_to_edge,
            }
        }
        
        # Send GPS data
        self.publisher.send_json(gps_data)
        
        # If outside geofence, also send an alert command
        if not self.inside_geofence:
            alert_cmd = {
                'type': 'geofence_alert',
                'alert_level': 'critical',
                'distance_outside': -self.distance_to_edge,
                'direction_to_center': self._bearing_to_center(),
                'command': 'return_to_geofence'
            }
            self.publisher.send_json(alert_cmd)
        elif self.distance_to_edge < self.warning_distance:
            # Also send warning if near boundary
            warning_cmd = {
                'type': 'geofence_alert',
                'alert_level': 'warning',
                'distance_to_edge': self.distance_to_edge,
                'direction_to_center': self._bearing_to_center(),
                'command': 'approaching_boundary'
            }
            self.publisher.send_json(warning_cmd)

    def _haversine_distance(self, lat1, lon1, lat2, lon2):
        """
        Calculate the great circle distance between two points 
        on the earth (specified in decimal degrees)
        """
        # Convert decimal degrees to radians
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        
        # Haversine formula
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = (math.sin(dlat/2)**2 + 
             math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2)
        c = 2 * math.asin(math.sqrt(a))
        # Radius of earth in meters
        r = 6371000
        return c * r

    def _bearing_to_center(self):
        """Calculate bearing from current position to geofence center"""
        if not self.latitude or not self.longitude:
            return 0
            
        lat1, lon1 = self.latitude, self.longitude
        lat2, lon2 = self.geofence_center
        
        # Convert to radians
        lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
        
        # Calculate bearing
        dlon = lon2 - lon1
        x = math.sin(dlon) * math.cos(lat2)
        y = (math.cos(lat1) * math.sin(lat2) - 
             math.sin(lat1) * math.cos(lat2) * math.cos(dlon))
        bearing = math.atan2(x, y)
        
        # Convert to degrees
        bearing = math.degrees(bearing)
        bearing = (bearing + 360) % 360
        
        return bearing

    def get_position(self):
        """Get current position as (latitude, longitude) tuple"""
        return (self.latitude, self.longitude) if self.latitude and self.longitude else None

    def get_geofence_status(self):
        """Get current geofence status"""
        return {
            'enabled': self.geofence_enabled,
            'inside': self.inside_geofence,
            'distance_to_center': self.distance_to_center,
            'distance_to_edge': self.distance_to_edge,
        }

    def cleanup(self):
        """Clean up resources"""
        self.stop()
        if hasattr(self, 'publisher'):
            self.publisher.close()
        if hasattr(self, 'context'):
            self.context.term()