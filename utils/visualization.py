# utils/visualization.py
import matplotlib.pyplot as plt
import numpy as np

class LidarVisualizer:
    def __init__(self):
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        self.scat = self.ax.scatter([], [])
        self.ax.set_rmax(3)  # 3 meter range
        plt.ion()

    def update(self, angles, distances, obstacles):
        # Clear previous plot
        self.ax.clear()
        
        # Convert to radians
        theta = np.radians(angles)
        
        # Plot raw data
        self.ax.scatter(theta, distances, s=2, c='blue')
        
        # Plot obstacles
        if obstacles:
            obs_theta = [np.arctan2(o['y'], o['x']) for o in obstacles]
            obs_r = [np.hypot(o['x'], o['y']) for o in obstacles]
            self.ax.scatter(obs_theta, obs_r, s=50, c='red', marker='x')
        
        self.ax.set_rmax(3)
        plt.draw()
        plt.pause(0.001)