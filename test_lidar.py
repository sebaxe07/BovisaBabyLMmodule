# test_lidar.py
from sensors.lidar_processor import LidarProcessor
import yaml
import time

def load_config():
    with open('config/settings.yaml') as f:
        return yaml.safe_load(f)

if __name__ == "__main__":
    config = load_config()
    print("Configuration loaded.")
    print("LIDAR Port:", config['lidar']['port'])
    print("Starting LIDAR processor...")
    processor = LidarProcessor(config['lidar'])
    
    try:
        processor.start()
        print("LIDAR running, press Ctrl+C to stop")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping...")
        processor.stop()