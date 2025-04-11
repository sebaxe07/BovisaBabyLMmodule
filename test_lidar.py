import yaml
from sensors.lidar_processor import LidarProcessor
from utils.visualization import LidarVisualizer
import multiprocessing
import time

def load_config():
    with open('config/settings.yaml') as f:
        return yaml.safe_load(f)

def run_lidar(config):
    processor = LidarProcessor(config['lidar'])
    processor.start()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        processor.stop()

if __name__ == "__main__":
    config = load_config()
    
    # Start LIDAR in a separate process
    lidar_process = multiprocessing.Process(
        target=run_lidar, 
        args=(config,)
    )
    lidar_process.start()
    
    # Start visualization in main process
    try:
        visualizer = LidarVisualizer(config['lidar'])
        visualizer.run()
    finally:
        lidar_process.terminate()
        lidar_process.join()