import matplotlib.pyplot as plt
import numpy as np
import zmq
import time

class LidarVisualizer:
    def __init__(self, config):
        self.config = config
        plt.ion()  # Turn on interactive mode
        self.fig, self.ax = plt.subplots(figsize=(8,8), subplot_kw={'projection': 'polar'})
        
        # Setup plots
        self.scan_plot = self.ax.scatter([], [], s=5, c='blue', alpha=0.5, label='Scan Points')
        self.obstacle_plot = self.ax.scatter([], [], s=100, c='red', marker='x', label='Obstacles')

        
        # Configure plot appearance
        self.ax.set_theta_zero_location('N')  # 0° at top
        self.ax.set_theta_direction(-1)  # Clockwise rotation
        # Set maximum range and ensure it's applied properly
        max_range = 5  # Set this to your desired maximum range in meters
        self.ax.set_rmax(max_range)
        self.ax.set_rlim(0, max_range) 
        self.ax.set_rticks([1, 2, 3, 4, 5])  # Add distance markers
        self.ax.grid(True, linestyle='--', alpha=0.5)
        
        # Add safety distance indicator
        safety_angle = np.linspace(0, 2*np.pi, 100)
        safety_dist = np.ones(100) * config['safety_distance']
        self.safety_line, = self.ax.plot(safety_angle, safety_dist, 'r--', linewidth=1, label='Safety Distance')
        
        # Add center point (robot position)
        self.center_point, = self.ax.plot(0, 0, 'go', markersize=8, label='Robot')
        
        self.ax.legend(loc='upper right')
        self.ax.set_title("LIDAR Obstacle Detection", pad=20)
        
        
        # Setup ZMQ
        self.context = zmq.Context()
        self.subscriber = self.context.socket(zmq.SUB)
        self.subscriber.connect("tcp://localhost:5555")
        self.subscriber.setsockopt_string(zmq.SUBSCRIBE, '')
        
        # Track last update time for FPS control
        self.last_update = time.time()
        
    def update(self):
        try:
            # Process all waiting messages
            while True:
                try:
                    msg = self.subscriber.recv_json(zmq.NOBLOCK)
                    if msg['type'] == 'obstacles':
                        self._process_obstacles(msg['data'])
                    if msg['type'] == 'scan':
                        angles = np.radians(msg['data']['angles'])
                        distances = np.array(msg['data']['distances'])
                        self.scan_plot.set_offsets(np.c_[angles, distances])
                        self.scan_plot.set_array(distances)
                        
                        # Update robot position
                        self.center_point.set_data(0, 0)
                except zmq.Again:
                    break
            
            # Throttle plot updates to ~30 FPS
            if time.time() - self.last_update > 0.033:
                plt.draw()
                plt.pause(0.001)  # Needed for interactive mode
                self.last_update = time.time()
                
            return True
            
        except Exception as e:
            print(f"Visualization error: {e}")
            return False
    
    def _process_obstacles(self, obstacles):
        """Update obstacle plot data"""
        if obstacles:
            angles = [np.arctan2(o['y'], o['x']) for o in obstacles]
            dists = [o['distance'] for o in obstacles]
            self.obstacle_plot.set_offsets(np.c_[angles, dists])
        else:
            self.obstacle_plot.set_offsets(np.empty((0, 2)))
    
    def run(self):
        """Main visualization loop"""
        try:
            while plt.fignum_exists(self.fig.number):
                if not self.update():
                    break
                time.sleep(0.01)
        except KeyboardInterrupt:
            pass
        finally:
            plt.close()
            self.subscriber.close()
            self.context.term()