# sensors/gps_processor.py
import time
import random
import math
import zmq
import serial
import threading
import pynmea2
import json
from threading import Thread, Event
from shapely.geometry import Point, Polygon
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
        self.has_fix = False  # Whether we have a valid GPS fix
        self.status_message = "No GPS data yet"  # Human-readable status
        
        # Geofence status
        self.geofence_enabled = config['geofence']['enabled']
        self.warning_distance = config['geofence']['warning_distance']
        self.inside_geofence = True
        self.distance_to_edge = 0.0
        
        # Load geofence from JSON file
        if self.geofence_enabled:
            self._load_geofence(config['geofence']['geofence_file'])
            
        # Flag to track first publish
        self.first_publish = False
        
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
                            if isinstance(msg, pynmea2.RMC):  # GNRMC, GPRMC
                                # RMC contains position, speed, course, and status
                                if msg.status == 'A':  # 'A' = valid, 'V' = void
                                    self.has_fix = True
                                    self.status_message = "GPS fix acquired"
                                    if hasattr(msg, 'latitude') and msg.latitude:
                                        self.latitude = msg.latitude
                                        self.longitude = msg.longitude
                                        self.last_update = time.time()
                                        self.speed = float(msg.spd_over_grnd) * 1.852 if msg.spd_over_grnd else 0.0  # Convert knots to km/h
                                        self.course = float(msg.true_course) if msg.true_course else 0.0
                                        self._update_geofence_status()
                                        self._publish_gps_data()
                                else:
                                    self.has_fix = False
                                    self.status_message = "No GPS fix (GNRMC: Void navigation data)"
                                    log_debug("GPS", "RMC message with void status")
                                
                            elif isinstance(msg, pynmea2.GGA):  # GNGGA, GPGGA
                                # GGA message contains position, altitude, and fix quality
                                self.fix_quality = int(msg.gps_qual) if msg.gps_qual else 0
                                self.satellites = int(msg.num_sats) if msg.num_sats else 0
                                
                                if self.fix_quality > 0 and msg.latitude and msg.longitude:
                                    self.has_fix = True
                                    self.status_message = f"GPS fix acquired (Quality: {self.fix_quality})"
                                    self.latitude = msg.latitude
                                    self.longitude = msg.longitude
                                    self.altitude = float(msg.altitude) if msg.altitude else None
                                    self.hdop = float(msg.horizontal_dil) if msg.horizontal_dil else 99.9
                                    self.last_update = time.time()
                                    
                                    # Calculate geofence status
                                    self._update_geofence_status()
                                    self._publish_gps_data()
                                else:
                                    self.has_fix = False
                                    if self.fix_quality == 0:
                                        self.status_message = "No GPS fix (GNGGA: Fix quality 0)"
                                    else:
                                        self.status_message = f"Invalid GGA data with quality {self.fix_quality}"
                                    log_debug("GPS", f"GGA message without fix. Quality: {self.fix_quality}, Satellites: {self.satellites}")
                                
                            elif isinstance(msg, pynmea2.GSA):  # GNGSA, GPGSA
                                # GSA contains DOP (dilution of precision) values and fix type
                                # Mode: 1=no fix, 2=2D fix, 3=3D fix
                                fix_type = int(msg.mode_fix) if hasattr(msg, 'mode_fix') and msg.mode_fix else 1
                                
                                if hasattr(msg, 'pdop'):
                                    pdop = float(msg.pdop) if msg.pdop else 99.99
                                    hdop = float(msg.hdop) if msg.hdop else 99.99
                                    vdop = float(msg.vdop) if msg.vdop else 99.99
                                    
                                    # Only update HDOP if better than current
                                    if hdop < self.hdop:
                                        self.hdop = hdop
                                
                                if fix_type == 1:
                                    log_debug("GPS", f"GSA indicates no fix (Mode: {fix_type})")
                                    if self.status_message == "No GPS data yet":
                                        self.status_message = "No GPS fix (GNGSA: Fix not available)"
                                elif fix_type >= 2:
                                    log_debug("GPS", f"GSA indicates {fix_type}D fix")
                            
                            elif isinstance(msg, pynmea2.GSV):  # GPGSV, GAGSV, GBGSV, GQGSV
                                # GSV contains satellites in view information
                                if hasattr(msg, 'num_sv_in_view') and msg.num_sv_in_view:
                                    sats_in_view = int(msg.num_sv_in_view)
                                    if sats_in_view > self.satellites:
                                        self.satellites = sats_in_view
                                    if sats_in_view == 0:
                                        log_debug("GPS", "GSV indicates no satellites in view")
                                        if self.status_message == "No GPS data yet":
                                            self.status_message = "No GPS fix (GSV: No satellites in view)"
                            
                            elif isinstance(msg, pynmea2.VTG):  # GNVTG, GPVTG
                                # VTG message contains speed and course
                                if hasattr(msg, 'spd_over_grnd_kmph') and msg.spd_over_grnd_kmph:
                                    self.speed = float(msg.spd_over_grnd_kmph)
                                if hasattr(msg, 'true_track') and msg.true_track:
                                    self.course = float(msg.true_track)
                            
                            elif isinstance(msg, pynmea2.GLL):  # GNGLL, GPGLL
                                # GLL contains position and status
                                status = msg.status if hasattr(msg, 'status') else 'V'
                                
                                if status == 'A' and msg.latitude and msg.longitude:
                                    self.has_fix = True
                                    self.latitude = msg.latitude
                                    self.longitude = msg.longitude
                                    self.last_update = time.time()
                                    log_debug("GPS", "Valid GLL position received")
                                else:
                                    log_debug("GPS", "GLL with void status")
                                    if self.status_message == "No GPS data yet":
                                        self.status_message = "No GPS fix (GNGLL: Void status)"
                            
                        except pynmea2.ParseError:
                            # Ignore parse errors - some sentences might be corrupt
                            pass
                            
                        # Check if fix has timed out (no valid position for >10 seconds)
                        if self.has_fix and time.time() - self.last_update > 10:
                            self.has_fix = False
                            self.status_message = "GPS fix lost (timeout)"
                            log_debug("GPS", "GPS fix timed out")
                            self._publish_gps_data()
                            
            except serial.SerialException as e:
                log_error("GPS", f"Serial error: {e}")
                # Wait before trying to reconnect
                self.status_message = f"GPS connection error: {e}"
                time.sleep(5)
                
            except Exception as e:
                log_error("GPS", f"Error in GPS loop: {e}")
                self.status_message = f"GPS processing error: {e}"
                time.sleep(1)

    def _update_geofence_status(self):
        """Update geofence status based on current position"""
        if not self.geofence_enabled or not self.latitude or not self.longitude or not hasattr(self, 'geofence_polygon') or self.geofence_polygon is None:
            return
            
        # Create a Point object for current position (Shapely uses lon/lat order)
        current_point = Point(self.longitude, self.latitude)
        
        # Check if inside geofence
        was_inside = self.inside_geofence
        self.inside_geofence = self.geofence_polygon.contains(current_point)
        
        # Calculate distance to geofence edge
        if self.inside_geofence:
            # If inside, calculate distance to nearest edge
            self.distance_to_edge = self._distance_to_polygon_edge(current_point, self.geofence_polygon)
        else:
            # If outside, distance is negative to indicate outside status
            self.distance_to_edge = -self._distance_to_polygon_edge(current_point, self.geofence_polygon)
        
        # Calculate distance to geofence center (for bearing calculation)
        self.distance_to_center = self._haversine_distance(
            self.latitude, self.longitude,
            self.geofence_center[0], self.geofence_center[1]
        )
        
        # Log geofence status changes
        if was_inside != self.inside_geofence:
            if self.inside_geofence:
                log_info("GPS", "Entered geofence area")
            else:
                log_error("GPS", f"LEFT GEOFENCE AREA! Distance to edge: {abs(self.distance_to_edge):.2f}m")
        elif not self.inside_geofence:
            log_error("GPS", f"Outside geofence by {abs(self.distance_to_edge):.2f}m")
        elif abs(self.distance_to_edge) < self.warning_distance:
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
            },
            'status_message': self.status_message
        }
        
        # Include polygon data if available (first time only to save bandwidth)
        if hasattr(self, 'geofence_data') and self.geofence_data and hasattr(self, 'first_publish') and not self.first_publish:
            gps_data['geofence']['geofence_data'] = self.geofence_data
            self.first_publish = True
        
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
        status = {
            'enabled': self.geofence_enabled,
            'inside': self.inside_geofence,
            'distance_to_center': self.distance_to_center,
            'distance_to_edge': self.distance_to_edge,
        }
        
        # Include polygon data if available 
        if hasattr(self, 'geofence_data') and self.geofence_data:
            status['polygon_type'] = self.geofence_data.get('type', 'Polygon')
            status['polygon_name'] = self.geofence_data.get('name', 'Default Geofence')
            
        return status

    def get_status_info(self):
        """
        Get detailed status information about the GPS connection.
        
        Returns:
            dict: A dictionary containing status information about the GPS connection
        """
        time_since_update = time.time() - self.last_update if self.last_update > 0 else float('inf')
        
        return {
            'has_fix': self.has_fix,
            'fix_quality': self.fix_quality,
            'satellites': self.satellites,
            'hdop': self.hdop,
            'status_message': self.status_message,
            'time_since_update': time_since_update,
            'geofence_status': self.get_geofence_status(),
            'connection_status': 'Connected' if time_since_update < 10 else 'Disconnected',
            'position': {
                'latitude': self.latitude,
                'longitude': self.longitude,
                'altitude': self.altitude,
                'speed': self.speed,
                'course': self.course
            }
        }
        
    def interpret_gps_status(self):
        """
        Provide a human-readable interpretation of the current GPS status.
        
        Returns:
            str: A detailed explanation of the GPS status
        """
        if not self.has_fix:
            causes = []
            if self.satellites == 0:
                causes.append("No satellites in view")
            if self.hdop >= 20:
                causes.append(f"Poor precision (HDOP: {self.hdop})")
            if self.status_message and "Void" in self.status_message:
                causes.append("Void navigation data")
            if self.fix_quality == 0:
                causes.append("No position fix")
                
            if causes:
                cause_text = ", ".join(causes)
                return f"No valid GPS position. Reason: {cause_text}. This may occur indoors or with obstructed sky view."
            else:
                return "No valid GPS position. Try moving to an open area with clear sky view."
        else:
            quality_desc = "Excellent" if self.hdop < 1.0 else "Good" if self.hdop < 2.0 else "Fair" if self.hdop < 5.0 else "Poor"
            return f"GPS fix acquired with {self.satellites} satellites. Signal quality: {quality_desc} (HDOP: {self.hdop:.1f})"

    def cleanup(self):
        """Clean up resources"""
        self.stop()
        if hasattr(self, 'publisher'):
            self.publisher.close()
        if hasattr(self, 'context'):
            self.context.term()

    def _load_geofence(self, geofence_file):
        """Load geofence polygon from JSON file"""
        try:
            with open(geofence_file, 'r') as f:
                geofence_data = json.load(f)
                
            # Store the geofence data
            self.geofence_data = geofence_data
            
            # Create a Shapely polygon from coordinates
            # Note: Shapely uses (lon, lat) order, but our JSON uses (lat, lon)
            # so we need to swap the coordinates
            polygon_points = [(lon, lat) for lat, lon in geofence_data['coordinates']]
            self.geofence_polygon = Polygon(polygon_points)
            
            # Store the center for distance calculations
            if 'center' in geofence_data:
                self.geofence_center = (geofence_data['center'][0], geofence_data['center'][1])
            else:
                # Calculate centroid if center not specified
                centroid = self.geofence_polygon.centroid
                self.geofence_center = (centroid.y, centroid.x)
                
            log_info("GPS", f"Loaded geofence with {len(geofence_data['coordinates'])} points from {geofence_file}")
            
        except Exception as e:
            log_error("GPS", f"Failed to load geofence from {geofence_file}: {e}")
            self.geofence_enabled = False
            self.geofence_polygon = None
            self.geofence_data = None

    def _distance_to_polygon_edge(self, point, polygon):
        """
        Calculate the minimum distance from a point to the edge of a polygon.
        
        Args:
            point: A shapely Point object
            polygon: A shapely Polygon object
            
        Returns:
            float: Distance in meters
        """
        # If the point is inside the polygon, return the distance to boundary
        if polygon.contains(point):
            return self._meters_to_degrees(polygon.boundary.distance(point))
        
        # If the point is outside, return distance to boundary with negative sign
        return self._meters_to_degrees(polygon.exterior.distance(point))
        
    def _meters_to_degrees(self, distance_degrees):
        """Convert a distance in decimal degrees to meters.
        
        This is an approximation that works well enough for small distances.
        1 degree of latitude is approximately 111,111 meters.
        1 degree of longitude varies with latitude.
        """
        # Average latitude for approximation
        avg_lat = self.latitude
        
        # One degree of latitude in meters (approximately)
        one_lat_degree_meters = 111111
        
        # One degree of longitude in meters (approximately, varies with latitude)
        one_lon_degree_meters = 111111 * math.cos(math.radians(avg_lat))
        
        # Use an average conversion factor
        avg_degree_meters = (one_lat_degree_meters + one_lon_degree_meters) / 2
        
        # Convert from degrees to meters
        return distance_degrees * avg_degree_meters