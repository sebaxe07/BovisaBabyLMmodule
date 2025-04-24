# sensors/lidar_processor.py
import logging
import random
import numpy as np
from pyrplidar import PyRPlidar
from collections import deque
from threading import Thread, Event
import time
import json
import zmq  # For message publishing
from sklearn.cluster import DBSCAN
from utils.colored_logger import log_info, log_error, log_debug

class LidarProcessor:
    def __init__(self, config):
        log_info("LIDAR", "Initializing LIDAR processor")
        self.config = config
        self.scan_buffer = deque(maxlen=5)
        self.obstacles = []
        self._setup_communication()
        self._running = Event()
        self._thread = None 

    def _setup_communication(self):
        context = zmq.Context()
        self.publisher = context.socket(zmq.PUB)
        self.publisher.bind("tcp://*:5555")  # Publish obstacle data

    def _process_scan(self, angles, distances):
        """Convert raw scan to obstacle data with tracking"""
        obstacles = []
        
        # Convert polar to cartesian
        x = np.array(distances) * np.cos(np.radians(angles))
        y = np.array(distances) * np.sin(np.radians(angles))
        
        # Basic obstacle detection
        for xi, yi, dist in zip(x, y, distances):
            if dist < self.config['safety_distance']:
                obstacles.append({
                    'x': xi,
                    'y': yi,
                    'distance': dist
                })
        
        # Use improved clustering
        clustered = self._improved_clustering(obstacles)
        
        # Apply tracking to clustered obstacles
        tracked = self._track_obstacles(clustered)
        return tracked

    def _track_obstacles(self, new_obstacles):
        """Track obstacles between consecutive scans"""
        # Initialize tracking state if first run
        if not hasattr(self, 'previous_obstacles'):
            self.previous_obstacles = new_obstacles
            # Assign initial IDs
            for i, obs in enumerate(self.previous_obstacles):
                obs['id'] = i
            return new_obstacles
        
        # Associate new obstacles with previous ones
        for new_obs in new_obstacles:
            min_dist = float('inf')
            matching_id = None
            
            # Find closest previous obstacle
            for prev_obs in self.previous_obstacles:
                dist = np.hypot(new_obs['x'] - prev_obs['x'], new_obs['y'] - prev_obs['y'])
                if dist < min_dist and dist < 0.5:  # Maximum association distance
                    min_dist = dist
                    matching_id = prev_obs.get('id')
                    
            if matching_id is not None:
                # This is a continuing obstacle
                new_obs['id'] = matching_id
                
                # Calculate velocity
                prev_matching = next(o for o in self.previous_obstacles if o.get('id') == matching_id)
                dt = self.config.get('scan_interval', 0.1)  # Time between scans
                new_obs['vx'] = (new_obs['x'] - prev_matching['x']) / dt
                new_obs['vy'] = (new_obs['y'] - prev_matching['y']) / dt
                new_obs['speed'] = np.hypot(new_obs['vx'], new_obs['vy'])
            else:
                # This is a new obstacle
                max_id = max([o.get('id', -1) for o in self.previous_obstacles] + 
                            [o.get('id', -1) for o in new_obstacles]) + 1
                new_obs['id'] = max_id
                new_obs['vx'] = 0
                new_obs['vy'] = 0
                new_obs['speed'] = 0
        
        # Update tracking state for next iteration
        self.previous_obstacles = new_obstacles.copy()
        return new_obstacles

    def _improved_clustering(self, obstacles):
        """Cluster obstacles using DBSCAN algorithm and return polygon representations"""
        if len(obstacles) < 3:
            return self._cluster_obstacles(obstacles)  # Fall back to original method
            
        # Extract points as numpy array
        points = np.array([[obs['x'], obs['y']] for obs in obstacles])
        
        # Make DBSCAN parameters adaptive based on point density
        if len(points) > 5:
            # Calculate average distance to nearest neighbors
            from scipy.spatial import KDTree
            kdtree = KDTree(points)
            distances, _ = kdtree.query(points, k=3)  # Find 3 nearest neighbors
            avg_nn_dist = np.mean(distances[:, 1:])  # Skip first one (self)
            
            # Adaptive parameters based on point density
            eps = max(0.15, min(0.5, avg_nn_dist * 2.0))
            min_samples = max(2, min(5, int(len(points) * 0.05)))  # 5% of points, between 2-5
            log_info("LIDAR", f"Adaptive DBSCAN: eps={eps:.2f}, min_samples={min_samples}")
        else:
            eps = 0.3
            min_samples = 2
        
        # Apply DBSCAN clustering
        db = DBSCAN(eps=eps, min_samples=min_samples).fit(points)
        labels = db.labels_
        
        # Process clusters
        clusters = []
        unique_labels = set(labels)
        for label in unique_labels:
            if label == -1:  # Skip noise points
                continue
                
            # Get points in this cluster
            mask = labels == label
            cluster_points = [obstacles[i] for i in range(len(obstacles)) if mask[i]]
            
            # Calculate cluster centroid properties
            x_avg = sum(p['x'] for p in cluster_points) / len(cluster_points)
            y_avg = sum(p['y'] for p in cluster_points) / len(cluster_points)
            min_dist = min(p['distance'] for p in cluster_points)
            
            # Calculate bounding box for the cluster
            x_values = [p['x'] for p in cluster_points]
            y_values = [p['y'] for p in cluster_points]
            min_x, max_x = min(x_values), max(x_values)
            min_y, max_y = min(y_values), max(y_values)
            
            # Ensure minimum size for small obstacles (like columns)
            width = max_x - min_x
            height = max_y - min_y
            min_size = 0.1  # Minimum 10cm size for any dimension
            
            if width < min_size:
                center_x = (min_x + max_x) / 2
                min_x = center_x - min_size / 2
                max_x = center_x + min_size / 2
                
            if height < min_size:
                center_y = (min_y + max_y) / 2
                min_y = center_y - min_size / 2
                max_y = center_y + min_size / 2
            
            # Create the enhanced obstacle representation
            clusters.append({
                'x': x_avg,            # Center x (for compatibility with existing code)
                'y': y_avg,            # Center y (for compatibility with existing code)
                'distance': min_dist,  # Distance to closest point (for compatibility)
                'polygon': {
                    'type': 'rectangle',
                    'corners': [
                        [min_x, min_y],  # Bottom-left
                        [max_x, min_y],  # Bottom-right
                        [max_x, max_y],  # Top-right
                        [min_x, max_y],  # Top-left
                    ]
                },
                'width': max_x - min_x,
                'height': max_y - min_y,
                'area': (max_x - min_x) * (max_y - min_y),
                'point_count': len(cluster_points)
            })
        
        return clusters
    
    def _cluster_obstacles(self, obstacles, dist_threshold=0.3):
        """Group nearby obstacles and return polygon representations"""
        clusters = []
        for obs in obstacles:
            matched = False
            for cluster in clusters:
                dx = obs['x'] - cluster['x']
                dy = obs['y'] - cluster['y']
                if np.hypot(dx, dy) < dist_threshold:
                    # Update centroid
                    cluster['x'] = (cluster['x'] + obs['x'])/2
                    cluster['y'] = (cluster['y'] + obs['y'])/2
                    cluster['distance'] = min(cluster['distance'], obs['distance'])
                    
                    # Update point list and recalculate bounding box
                    if 'points' not in cluster:
                        cluster['points'] = [{'x': cluster['x'], 'y': cluster['y']}]
                    cluster['points'].append({'x': obs['x'], 'y': obs['y']})
                    
                    # Recalculate bounding box
                    x_values = [p['x'] for p in cluster['points']]
                    y_values = [p['y'] for p in cluster['points']]
                    min_x, max_x = min(x_values), max(x_values)
                    min_y, max_y = min(y_values), max(y_values)
                    
                    # Ensure minimum size
                    min_size = 0.1
                    width = max_x - min_x
                    height = max_y - min_y
                    
                    if width < min_size:
                        center_x = (min_x + max_x) / 2
                        min_x = center_x - min_size / 2
                        max_x = center_x + min_size / 2
                        
                    if height < min_size:
                        center_y = (min_y + max_y) / 2
                        min_y = center_y - min_size / 2
                        max_y = center_y + min_size / 2
                    
                    # Update polygon info
                    cluster['polygon'] = {
                        'type': 'rectangle',
                        'corners': [
                            [min_x, min_y],  # Bottom-left
                            [max_x, min_y],  # Bottom-right
                            [max_x, max_y],  # Top-right
                            [min_x, max_y],  # Top-left
                        ]
                    }
                    cluster['width'] = max_x - min_x
                    cluster['height'] = max_y - min_y
                    cluster['area'] = (max_x - min_x) * (max_y - min_y)
                    cluster['point_count'] = len(cluster['points'])
                    
                    matched = True
                    break
            
            if not matched:
                new_cluster = obs.copy()
                # Add polygon info for new single-point clusters
                min_size = 0.1
                new_cluster['polygon'] = {
                    'type': 'rectangle',
                    'corners': [
                        [obs['x'] - min_size/2, obs['y'] - min_size/2],
                        [obs['x'] + min_size/2, obs['y'] - min_size/2],
                        [obs['x'] + min_size/2, obs['y'] + min_size/2],
                        [obs['x'] - min_size/2, obs['y'] + min_size/2],
                    ]
                }
                new_cluster['width'] = min_size
                new_cluster['height'] = min_size
                new_cluster['area'] = min_size * min_size
                new_cluster['point_count'] = 1
                new_cluster['points'] = [{'x': obs['x'], 'y': obs['y']}]
                clusters.append(new_cluster)
                
        return clusters
    
    def _scan_loop(self):
        # Check if we're in mock mode
        if self.config.get('mock_mode', False):
            log_info("LIDAR", "Running in mock mode")
            self._mock_scan_loop()
            return

        lidar = PyRPlidar()
        attempt = 0
        max_attempts = 3

        # Scan quality check
        expected_points = 1050  # Expected points per scan (~1060-1070)
        min_acceptable = 100    # Minimum acceptable points threshold
        point_history = []      # Keep track of recent scan quality
        history_size = 5        # Number of scans to track
        consecutive_bad_scans = 0
        max_bad_scans = 3       # Trigger reset after this many bad scans
        
        
        while self._running.is_set():
            try:
                attempt += 1
                log_info("LIDAR", f"Connecting to LIDAR (Attempt {attempt}/{max_attempts})...")
                lidar.connect(port=self.config['port'], baudrate=115200, timeout=3)
                lidar.set_motor_pwm(500)
                time.sleep(2)

                scan_generator = lidar.start_scan_express(mode=2)
                current_scan = {'angles': [], 'distances': []}
                
                # Reset bad scan counter on successful reconnection
                consecutive_bad_scans = 0
                
                for scan in scan_generator():
                    if not self._running.is_set():
                        break

                    if scan.start_flag:
                        if current_scan['angles']:
                            # Check scan quality
                            points_count = len(current_scan['angles'])
                            
                            # Log point count periodically
                            if random.random() < 0.05:  # Log ~5% of scans
                                log_info("LIDAR", f"Scan points: {points_count}")
                            
                            # Track scan quality
                            point_history.append(points_count)
                            if len(point_history) > history_size:
                                point_history.pop(0)
                            
                            # Check if scan quality is deteriorating
                            if points_count < min_acceptable:
                                consecutive_bad_scans += 1
                                log_error("LIDAR", f"Low quality scan detected: {points_count} points (threshold: {min_acceptable})")
                                
                                if consecutive_bad_scans >= max_bad_scans:
                                    log_error("LIDAR", f"Multiple bad scans detected ({consecutive_bad_scans}), resetting LIDAR...")
                                    raise Exception("LIDAR quality degraded, forcing reset")
                            else:
                                consecutive_bad_scans = 0  # Reset counter on good scan
                            
                            # Process the scan normally
                            obstacles = self._process_scan(
                                current_scan['angles'],
                                current_scan['distances']
                            )
                            self.publisher.send_json({
                                'type': 'obstacles',
                                'data': obstacles
                            })
                            self.publisher.send_json({
                                'type': 'scan',
                                'data': {
                                    'angles': current_scan['angles'],
                                    'distances': current_scan['distances'],
                                    'quality': points_count
                                }
                            })
                            
                            # Reset for next scan
                            current_scan = {'angles': [], 'distances': []}
                        
                    if scan.distance > 0:
                        current_scan['angles'].append(scan.angle)
                        current_scan['distances'].append(scan.distance/1000.0)
                
                log_info("LIDAR", "Scan loop stopped")
                
            except Exception as e:
                log_error("LIDAR", f"Error: {e}")
                
                # Enhanced cleanup before retry
                try:
                    lidar.stop()
                    lidar.set_motor_pwm(0)
                    lidar.disconnect()
                except:
                    pass
                    
                # Force USB reset - add more aggressive recovery
                if "quality degraded" in str(e):
                    log_info("LIDAR", "Performing extended recovery sequence")
                    time.sleep(3)  # Extended wait for USB to stabilize
                else:
                    time.sleep(1)
                    
                # If we exceed max attempts, wait longer before trying again
                if attempt >= max_attempts:
                    log_info("LIDAR", "Maximum attempts reached, waiting before retrying...")
                    time.sleep(10)
                    attempt = 0
                    
            finally:
                try:
                    log_info("LIDAR", "Disconnecting LIDAR")
                    lidar.stop()
                    lidar.set_motor_pwm(0)
                    time.sleep(0.5)
                    lidar.disconnect()
                    # Force an additional wait after disconnect
                    time.sleep(1)
                except Exception as e:
                    log_error("LIDAR", f"Error during cleanup: {e}")

    def _mock_scan_loop(self):
        """Generate fake LIDAR data when in mock mode"""
        log_info("LIDAR", "Starting mock LIDAR scan loop")
        
        while self._running.is_set():
            # Generate mock scan data
            scan_points = 180  # One point per 2 degrees
            angles = np.linspace(0, 359, scan_points)
            
            # Generate varying distances - simulating a room with occasional obstacles
            base_distances = np.ones(scan_points) * 3.0  # 3m base distance
            
            # Add some random walls/obstacles
            for _ in range(3):
                start_angle = random.randint(0, scan_points - 20)
                length = random.randint(10, 30)
                distance = random.uniform(0.3, 2.0)
                end_angle = min(start_angle + length, scan_points)
                base_distances[start_angle:end_angle] = distance
            
            # Add random noise
            distances = base_distances + np.random.normal(0, 0.05, scan_points)
            distances = np.clip(distances, 0.1, 8.0)  # Constrain to realistic values
            
            # Process the mock scan
            obstacles = self._process_scan(angles.tolist(), distances.tolist())
            
            # Publish the mock data
            self.publisher.send_json({
                'type': 'obstacles',
                'data': obstacles
            })
            self.publisher.send_json({
                'type': 'scan',
                'data': {
                    'angles': angles.tolist(),
                    'distances': distances.tolist(),
                    'quality': scan_points,
                    'mock': True
                }
            })
            
            # Simulate the scan interval
            time.sleep(self.config.get('scan_interval', 0.1))

    def start(self):
        self._running.set()
        self._thread = Thread(target=self._scan_loop)
        self._thread.start()

    def stop(self):
        self._running.clear()
        self._thread.join()

# Usage:
# processor = LidarProcessor(config['lidar'])
# processor.start()