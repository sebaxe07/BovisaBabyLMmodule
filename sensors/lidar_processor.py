# sensors/lidar_processor.py
import logging
import numpy as np
from pyrplidar import PyRPlidar
from collections import deque
from threading import Thread, Event
import time
import json
import zmq  # For message publishing

class LidarProcessor:
    def __init__(self, config):
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
        """Convert raw scan to obstacle data"""
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
        
        # Cluster obstacles (simple threshold-based clustering)
        clustered = self._cluster_obstacles(obstacles)
        return clustered

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
        try:
            print("Connecting to LIDAR...")
            print("LIDAR Port:", self.config['port'])
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
                        current_scan = {'angles': [], 'distances': []}
                    
                if scan.distance > 0:
                    current_scan['angles'].append(scan.angle)
                    current_scan['distances'].append(scan.distance/1000.0)  # Convert to meters

        except Exception as e:
            logging.error(f"LIDAR error: {e}")
        finally:
            lidar.stop()
            lidar.set_motor_pwm(0)
            lidar.disconnect()

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