import zmq
from threading import Thread
import time
import math
from utils.colored_logger import log_info, log_error, log_debug

# Global command publisher
command_publisher = None

def setup_command_publisher():
    """Initialize ZMQ command publisher"""
    global command_publisher
    context = zmq.Context()
    command_publisher = context.socket(zmq.PUB)
    command_publisher.bind("tcp://*:5556")  # For commands
    log_info("FLASK", "Command publisher initialized on port 5556")
    return command_publisher

def zmq_lidar_listener(scan_data, data_lock):
    """Listen for LIDAR scan data"""
    context = zmq.Context()
    subscriber = context.socket(zmq.SUB)
    subscriber.connect("tcp://localhost:5555")
    subscriber.setsockopt_string(zmq.SUBSCRIBE, '')
    
    log_info("FLASK", "ZMQ subscriber connected to LIDAR data stream")
    last_log_time = 0
    while True:
        try:
            msg = subscriber.recv_json()
            with data_lock:
                if msg['type'] == 'scan':
                    scan_data['angles'] = [float(a) for a in msg['data']['angles']]
                    scan_data['distances'] = [float(d) for d in msg['data']['distances']]
                    scan_data['cycle_count'] += 1

                    log_debug("FLASK", f"Updated scan data - cycle: {scan_data['cycle_count']}")
                    # Log periodically to avoid flooding
                    current_time = time.time()
                    if current_time - last_log_time > 5:  # Log every 5 seconds
                        last_log_time = current_time

                elif msg['type'] == 'obstacles':
                    scan_data['obstacles'] = msg['data']
                
        except Exception as e:
            log_error("FLASK", f"Error receiving LIDAR data: {e}")
            time.sleep(0.1)

def camera_to_lidar_coords(x_position, distance, camera_fov=70):
    """Convert camera coordinates to LIDAR coordinates"""
    max_x_value = 10.0
    angle_rad = (x_position / max_x_value) * (camera_fov / 2) * (math.pi / 180)
    
    x = distance * math.cos(angle_rad)
    y = distance * math.sin(angle_rad)
    
    return x, y

def zmq_camera_data_listener(scan_data, data_lock):
    """Listen for camera tracking data"""
    context = zmq.Context()
    camera_subscriber = context.socket(zmq.SUB)
    camera_subscriber.connect("tcp://localhost:5558")
    camera_subscriber.setsockopt_string(zmq.SUBSCRIBE, '')
    
    log_info("FLASK", "Camera subscriber connected to camera data stream")
    
    while True:
        try:
            camera_msg = camera_subscriber.recv_json()
            
            if camera_msg.get('type') == 'TRACKING':
                with data_lock:
                    x_position = float(camera_msg.get('x_position', 0))
                    distance = float(camera_msg.get('distance', 0))
                    
                    x, y = camera_to_lidar_coords(x_position, distance)
                    
                    angle = (math.degrees(math.atan2(y, x)) + 360) % 360
                    
                    scan_data["human"] = {
                        "x": x,
                        "y": y,
                        "distance": math.sqrt(x*x + y*y),
                        "angle": angle,
                        "timestamp": time.time()
                    }
                    
        except Exception as e:
            log_error("FLASK", f"Error receiving camera data: {e}")
            time.sleep(0.1)

def zmq_video_stream_listener(scan_data, data_lock):
    """Listen for video stream frames from camera client"""
    context = zmq.Context()
    video_subscriber = context.socket(zmq.SUB)
    video_subscriber.connect("tcp://localhost:5559")  # Connect to camera's video port
    video_subscriber.setsockopt_string(zmq.SUBSCRIBE, b"frame")
    
    log_info("FLASK", "Video subscriber connected to camera video stream")
    
    # Initialize frame buffer in scan_data
    with data_lock:
        scan_data["latest_frame"] = None
        scan_data["frame_timestamp"] = 0
    
    while True:
        try:
            # Receive multipart message (topic, frame bytes, timestamp)
            topic, frame_data, timestamp = video_subscriber.recv_multipart()
            
            with data_lock:
                scan_data["latest_frame"] = frame_data
                scan_data["frame_timestamp"] = int(timestamp)
                
        except Exception as e:
            log_error("FLASK", f"Error receiving video frame: {e}")
            time.sleep(0.1)

def start_zmq_listeners(scan_data, data_lock):
    """Start all ZMQ listener threads"""
    # Setup command publisher
    setup_command_publisher()
    
    # Start LIDAR data listener
    lidar_thread = Thread(target=zmq_lidar_listener, args=(scan_data, data_lock))
    lidar_thread.daemon = True
    lidar_thread.start()
    log_info("FLASK", "LIDAR data listener started")
    
    # Start camera data listener
    camera_thread = Thread(target=zmq_camera_data_listener, args=(scan_data, data_lock))
    camera_thread.daemon = True
    camera_thread.start()
    log_info("FLASK", "Camera data listener started")
    
    # # Start video stream listener
    # video_thread = Thread(target=zmq_video_stream_listener, args=(scan_data, data_lock))
    # video_thread.daemon = True
    # video_thread.start()
    # log_info("FLASK", "Video stream listener started")