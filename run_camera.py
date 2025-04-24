#!/usr/bin/env python3
import yaml
import time
import sys
import os
import signal
from multiprocessing import Process
from utils.colored_logger import log_info, log_error, log_debug

def load_config():
    with open('config/settings.yaml', 'r') as f:
        return yaml.safe_load(f)

def run_camera(config):
    log_info("MAIN", "Starting camera client...")
    from sensors.camera_client import CameraClient
    camera = CameraClient(config['camera'])
    camera.process_commands()  # Start processing camera commands

if __name__ == "__main__":
    # Load configuration
    try:
        config = load_config()
        log_info("MAIN", "Configuration loaded successfully")
    except Exception as e:
        log_error("MAIN", f"Error loading configuration: {e}")
        sys.exit(1)
    
    # Camera process needs to run in the main thread due to OpenCV display requirements
    try:
        run_camera(config)
    except KeyboardInterrupt:
        log_info("MAIN", "Keyboard interrupt received, shutting down camera...")
    except Exception as e:
        log_error("MAIN", f"Unexpected error: {e}")
    
    log_info("MAIN", "Camera process terminated")