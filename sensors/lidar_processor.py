# sensors/lidar_processor.py
import logging
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
        """Cluster obstacles using DBSCAN algorithm"""
        if len(obstacles) < 3:
            return self._cluster_obstacles(obstacles)  # Fall back to original method
            
        # Extract points as numpy array
        points = np.array([[obs['x'], obs['y']] for obs in obstacles])
        
        # Apply DBSCAN clustering
        db = DBSCAN(eps=0.3, min_samples=3).fit(points)
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
            
            # Calculate cluster properties
            x_avg = sum(p['x'] for p in cluster_points) / len(cluster_points)
            y_avg = sum(p['y'] for p in cluster_points) / len(cluster_points)
            min_dist = min(p['distance'] for p in cluster_points)
            
            clusters.append({
                'x': x_avg,
                'y': y_avg,
                'distance': min_dist
            })
        
        return clusters

    def _cluster_obstacles(self, obstacles, dist_threshold=0.3):
        """Group nearby obstacles"""
        clusters = []
        for obs in obstacles:
            matched = False
            for cluster in clusters:
                dx = obs['x'] - cluster['x']
                dy = obs['y'] - cluster['y']
                if np.hypot(dx, dy) < dist_threshold:
                    cluster['x'] = (cluster['x'] + obs['x'])/2
                    cluster['y'] = (cluster['y'] + obs['y'])/2
                    cluster['distance'] = min(cluster['distance'], obs['distance'])
                    matched = True
                    break
            if not matched:
                clusters.append(obs.copy())
        return clusters

    def _scan_loop(self):
        lidar = PyRPlidar()
        attempt = 0
        max_attempts = 3
        
        while attempt < max_attempts and self._running.is_set():
            try:
                attempt += 1
                log_info("LIDAR","Connecting to LIDAR...")
                log_info("LIDAR", f"LIDAR Port: {self.config['port']}")
                lidar.connect(port=self.config['port'], 
                            baudrate=115200, timeout=3)
                lidar.set_motor_pwm(500)
                time.sleep(2)

                scan_generator = lidar.start_scan_express(mode=2)
                current_scan = {'angles': [], 'distances': []}

                for scan in scan_generator():
                    if not self._running.is_set():
                        break

                    if scan.start_flag:
                        if current_scan['angles']:
                            # Process complete scan
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
                                    'distances': current_scan['distances']
                                }
                            })
                            current_scan = {'angles': [], 'distances': []}
                        
                    if scan.distance > 0:
                        current_scan['angles'].append(scan.angle)
                        current_scan['distances'].append(scan.distance/1000.0)  # Convert to meters
                
                log_info("LIDAR", "Scan loop stopped")
            except Exception as e:
                log_error("LIDAR", f"Error: {e}")
                time.sleep(1)  # Wait before retrying
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