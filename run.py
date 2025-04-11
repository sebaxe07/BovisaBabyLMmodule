import yaml
import time
import subprocess
import sys
import os
import signal
from multiprocessing import Process
from utils.colored_logger import log_info, log_error, log_debug

def load_config():
    with open('config/settings.yaml', 'r') as f:
        return yaml.safe_load(f)

def run_visualization(config):
    log_info("MAIN", "Starting visualization...")
    from utils.flask_visualization import app
    app.run(host='0.0.0.0', port=5000, debug=False)

def run_controller(config):
    log_info("MAIN", "Starting controller...")
    from controllers.main_controller import MainController
    controller = MainController(config)
    controller.process_messages()  # Start the message processing loop

if __name__ == "__main__":
    # Load configuration
    try:
        config = load_config()
        log_info("MAIN", "Configuration loaded successfully")
    except Exception as e:
        log_error("MAIN", f"Error loading configuration: {e}")
        sys.exit(1)
    
    # Create processes
    processes = [
        Process(target=run_visualization, args=(config,)),
        Process(target=run_controller, args=(config,))
    ]
    
    # Start all processes
    for p in processes:
        p.start()
        
    log_info("MAIN", "All processes started")
    
    # Wait for keyboard interrupt
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        log_info("MAIN", "Keyboard interrupt received, shutting down...")
        
    # Terminate all processes
    for p in processes:
        p.terminate()
    
    # Wait for processes to terminate
    for p in processes:
        p.join()
    
    log_info("MAIN", "All processes terminated")