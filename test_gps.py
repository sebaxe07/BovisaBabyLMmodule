#!/usr/bin/env python3
# test_gps.py
import yaml
import time
import sys
from sensors.gps_processor import GpsProcessor
from utils.colored_logger import log_info, log_error

def main():
    """Test the GPS processor module with mock data"""
    # Load configuration
    try:
        with open('config/settings.yaml', 'r') as f:
            config = yaml.safe_load(f)
    except Exception as e:
        log_error("TEST_GPS", f"Error loading config: {e}")
        sys.exit(1)
    
    # Create and start GPS processor
    gps = GpsProcessor(config['gps'])
    try:
        gps.start()
        log_info("TEST_GPS", "GPS processor started. Press Ctrl+C to stop.")
        
        # Run for a while to display GPS data
        while True:
            position = gps.get_position()
            status = gps.get_geofence_status()
            
            if position:
                lat, lon = position
                log_info("TEST_GPS", f"Position: {lat:.6f}, {lon:.6f} | " 
                         f"Inside Geofence: {status['inside']} | "
                         f"Distance to edge: {status['distance_to_edge']:.2f}m")
            else:
                log_error("TEST_GPS", "No position data available")
                
            time.sleep(1)
            
    except KeyboardInterrupt:
        log_info("TEST_GPS", "Test terminated by user")
    finally:
        gps.cleanup()
        log_info("TEST_GPS", "GPS processor cleaned up")

if __name__ == "__main__":
    main()