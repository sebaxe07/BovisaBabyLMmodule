import math
from flask import Flask, Response, render_template, render_template_string, jsonify, request
import zmq
from threading import Thread, Lock
import time
import json
import numpy as np
import logging
from utils.colored_logger import log_info, log_error, log_debug
import threading

# Suppress Flask's default logger
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

app = Flask(__name__, 
            template_folder='templates',
            static_folder='static')
VERBOSE = False  # Set to True for debugging

# Thread-safe data storage with default values
scan_data = {
    "angles": [0, 90, 180, 270],  # Default values for testing
    "distances": [1.0, 2.0, 3.0, 4.0],  # Default values
    "obstacles": [{"x": 1.0, "y": 0.0}],  # Default obstacle
    "cycle_count": 0,
    "max_distance": 5.0
}
data_lock = Lock()

command_publisher = None
def setup_command_publisher():
    global command_publisher
    context = zmq.Context()
    command_publisher = context.socket(zmq.PUB)
    command_publisher.bind("tcp://*:5556")  # Different port for commands
    log_info("FLASK", "Command publisher initialized on port 5556")


def log(message):
    if VERBOSE:
        log_info("FLASK", message)



def generate_frames():
    """Generate video frames from frame_data"""
    while True:
        with frame_lock:
            if frame_data.get("latest_frame") is not None:
                img_data = frame_data["latest_frame"]
                
                # Create MJPEG frame format
                yield (b'--frame\r\n'
                      b'Content-Type: image/jpeg\r\n\r\n' + img_data + b'\r\n')
                
        # Control the frame rate
        time.sleep(0.05)  # ~20 FPS



@app.route('/')
def index():
    return render_template('index.html')


@app.route('/video_feed')
def video_feed():
    """Video streaming route"""
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/data')
def get_data():
    with data_lock:
        # Return current data with debug info
        log(f"Serving data: {len(scan_data['angles'])} points, {len(scan_data['obstacles'])} obstacles")
        return jsonify(scan_data)
    
@app.route('/control/tracking')
def start_tracking():
    if command_publisher:
        command_publisher.send_json({
            'command': 'set_state',
            'state': 'TRACKING'
        })
        log_info("FLASK", "Sent command: Start tracking")
    return jsonify({'success': True, 'mode': 'TRACKING'})

@app.route('/control/stop')
def stop_robot():
    if command_publisher:
        command_publisher.send_json({
            'command': 'set_state',
            'state': 'IDLE'
        })
        command_publisher.send_json({
            'command': 'motor',
            'action': 'stop'
        })
        log_info("FLASK", "Sent command: Stop robot")
    return jsonify({'success': True, 'mode': 'IDLE'})

@app.route('/control/search')
def start_search():
    if command_publisher:
        command_publisher.send_json({
            'command': 'set_state',
            'state': 'SEARCH'
        })
        log_info("FLASK", "Sent command: Start search")
    return jsonify({'success': True, 'mode': 'SEARCH'})


@app.route('/control/update_human_position')
def update_human_position():
    """Update the mock human's position via ZMQ command"""
    x = float(request.args.get('x', -5.0))
    distance = float(request.args.get('distance', 2.0))
    
    if command_publisher:
        command_publisher.send_json({
            'command': 'UPDATE_HUMAN_POSITION',
            'x_position': x,
            'distance': distance
        })
        log_info("FLASK", f"Sent human position update: x={x}, distance={distance}")
    
    return jsonify({
        'success': True,
        'x_position': x,
        'distance': distance
    })


def zmq_listener():
    context = zmq.Context()
    subscriber = context.socket(zmq.SUB)
    subscriber.connect("tcp://localhost:5555")
    subscriber.setsockopt_string(zmq.SUBSCRIBE, '')
    
    log("ZMQ subscriber connected to LIDAR data stream")
    
    while True:
        try:
            msg = subscriber.recv_json()
            log(f"Received message type: {msg['type']}")
            
            with data_lock:
                if msg['type'] == 'scan':
                    scan_data['angles'] = [float(a) for a in msg['data']['angles']]
                    scan_data['distances'] = [float(d) for d in msg['data']['distances']]
                    scan_data['cycle_count'] += 1
                    log(f"Updated scan data (Cycle {scan_data['cycle_count']})")
                    
                elif msg['type'] == 'obstacles':
                    scan_data['obstacles'] = msg['data']
                    log(f"Updated obstacles: {len(msg['data'])} found")
                
        except Exception as e:
            log_error("ZMQ", f"Error receiving data: {e}")
            time.sleep(0.1)


def camera_to_lidar_coords(x_position, distance, camera_fov=70):
    """
    Convert camera coordinates to LIDAR coordinates
    - x_position: lateral position from camera (-10 to +10, negative=left, positive=right)
    - distance: forward distance in meters
    - camera_fov: camera's field of view in degrees
    
    Returns (x, y) in LIDAR coordinate system where:
    - x is forward distance
    - y is lateral position (negative=left, positive=right)
    """
    # Calculate the angle in radians
    # Assuming x_position of ±10 corresponds to ±(camera_fov/2) degrees
    max_x_value = 10.0
    angle_rad = (x_position / max_x_value) * (camera_fov / 2) * (math.pi / 180)
    
    # Calculate x,y coordinates
    x = distance * math.cos(angle_rad)
    y = distance * math.sin(angle_rad)
    
    # Since both camera and LIDAR have same conventions (right=positive),
    # we DON'T need to negate y
    return x, y

def camera_data_listener():
    """Listen for camera tracking data"""
    context = zmq.Context()
    camera_subscriber = context.socket(zmq.SUB)
    camera_subscriber.connect("tcp://localhost:5558")
    camera_subscriber.setsockopt_string(zmq.SUBSCRIBE, '')
    
    log_info("FLASK", "Camera subscriber connected to camera data stream")
    
    # Add to global scan_data
    global scan_data
    scan_data["human"] = None
    
    while True:
        try:
            camera_msg = camera_subscriber.recv_json()
            
            if camera_msg.get('type') == 'TRACKING':
                with data_lock:
                    # Get camera values
                    x_position = float(camera_msg.get('x_position', 0))
                    distance = float(camera_msg.get('distance', 0))
                    
                    # Use the improved coordinate conversion function
                    x, y = camera_to_lidar_coords(x_position, distance)
                    
                    # Calculate angle in polar coordinates (correct for LIDAR)
                    # For polar coordinates: 0° is forward, 90° is right
                    angle = (math.degrees(math.atan2(y, x)) + 360) % 360
                    
                    # Store human data for visualization
                    scan_data["human"] = {
                        "x": x,
                        "y": y,
                        "distance": math.sqrt(x*x + y*y),
                        "angle": angle,
                        "timestamp": time.time()
                    }
                    
                    #log_info("FLASK", f"Updated human position: distance={distance:.2f}m, angle={angle:.1f}°")
                    
        except Exception as e:
            log_error("FLASK", f"Error receiving camera data: {e}")
            time.sleep(0.1)

# Add this near the top with your other global variables
frame_data = {
    "latest_frame": None,
    "timestamp": 0
}
frame_lock = Lock()

def video_stream_listener():
    """Listen for video stream frames from camera client"""
    context = zmq.Context()
    video_subscriber = context.socket(zmq.SUB)
    video_subscriber.connect("tcp://localhost:5559")  # Connect to camera's video port
    video_subscriber.setsockopt(zmq.SUBSCRIBE, b"frame")  # Use setsockopt for bytes
    
    log_info("FLASK", "Video subscriber connected to camera video stream")
    
    while True:
        try:
            # Receive multipart message (topic, frame bytes, timestamp)
            topic, img_data, timestamp = video_subscriber.recv_multipart()
            
            with frame_lock:
                frame_data["latest_frame"] = img_data
                frame_data["timestamp"] = int(timestamp)
                
        except Exception as e:
            log_error("FLASK", f"Error receiving video frame: {e}")
            time.sleep(0.1)

# Create a function to initialize everything
def initialize():
    # Start ZMQ listener thread
    zmq_thread = Thread(target=zmq_listener)
    zmq_thread.daemon = True
    zmq_thread.start()
    setup_command_publisher()
    log_info("FLASK", "ZMQ listener started")
    camera_thread = Thread(target=camera_data_listener)
    camera_thread.daemon = True
    camera_thread.start()
    log_info("FLASK", "Camera data listener started")
    # Start video stream listener
    video_thread = Thread(target=video_stream_listener)
    video_thread.daemon = True
    video_thread.start()
    log_info("FLASK", "Video stream listener started")

# Initialize when module is imported
initialize()



if __name__ == '__main__':
    # Start Flask app directly if this script is executed
    log_info("FLASK", "Starting Flask server...")
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)