import math
from flask import Flask, render_template_string, jsonify, request
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

app = Flask(__name__)
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


# All-in-one HTML with robust visualization
HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>LIDAR Live View</title>
    <script src="https://cdn.plot.ly/plotly-latest.min.js"></script>
    <style>
        body { font-family: Arial, sans-serif; margin: 20px; }
        #plot { width: 100%; height: 80vh; border: 1px solid #ddd; }
        #status { 
            padding: 10px; 
            background: #f5f5f5; 
            margin-bottom: 10px;
            border-radius: 4px;
            font-family: monospace;
        }
        #controls {
            margin: 15px 0;
            display: flex;
            gap: 10px;
            align-items: center;
        }
        .control-button {
            padding: 10px 20px;
            font-size: 16px;
            border-radius: 4px;
            border: none;
            cursor: pointer;
            transition: all 0.3s;
        }
        .start-button { background: #4CAF50; color: white; }
        .stop-button { background: #f44336; color: white; }
        .sim-button { background: #2196F3; color: white; }
        .control-button:hover { opacity: 0.8; }
        .control-button:disabled { opacity: 0.5; cursor: not-allowed; }
    </style>
</head>
<body>
    <h1>LIDAR Real-Time Visualization</h1>
    <div id="status">Status: Waiting for initial data...</div>
    
    <!-- Control Panel -->
    <div id="controls">
        <button id="startTracking" class="control-button start-button">Start Tracking</button>
        <button id="stopTracking" class="control-button stop-button">Stop Robot</button>
        <button id="startSearch" class="control-button sim-button">Start Search</button>
        <span id="current-mode" style="margin-left: 20px; font-weight: bold;">Mode: IDLE</span>
    </div>



    <div id="simulation-controls" style="margin: 15px 0; padding: 15px; background: #f5f5f5; border-radius: 4px;">
        <h3>Mock Human Position Control</h3>
        <div style="margin: 15px 0;">
            <label for="x-position">X Position: <span id="x-value">-5.0</span></label>
            <input type="range" id="x-position" min="-10" max="10" step="0.1" value="-5.0" style="width: 80%;">
        </div>
        <div style="margin: 15px 0;">
            <label for="distance">Distance: <span id="distance-value">2.0</span></label>
            <input type="range" id="distance" min="0.5" max="10" step="0.1" value="2.0" style="width: 80%;">
        </div>
    </div>
    
    <div id="plot"></div>

    <script>
        // Initialize plot
        const plotDiv = document.getElementById('plot');
        const statusDiv = document.getElementById('status');
        const currentModeSpan = document.getElementById('current-mode');
        
        let updateTimeout = null;
        const THROTTLE_DELAY = 100; // milliseconds

        document.getElementById('x-position').addEventListener('input', function() {
            const xPosition = this.value;
            const distance = document.getElementById('distance').value;
            document.getElementById('x-value').textContent = xPosition;
            
            // Throttle the updates
            clearTimeout(updateTimeout);
            updateTimeout = setTimeout(() => {
                updateHumanPosition(xPosition, distance);
            }, THROTTLE_DELAY);
        });

        document.getElementById('distance').addEventListener('input', function() {
            const distance = this.value;
            const xPosition = document.getElementById('x-position').value;
            document.getElementById('distance-value').textContent = distance;
            
            // Throttle the updates
            clearTimeout(updateTimeout);
            updateTimeout = setTimeout(() => {
                updateHumanPosition(xPosition, distance);
            }, THROTTLE_DELAY);
        });

        // Create a function to handle the update
        function updateHumanPosition(x, distance) {
            fetch(`/control/update_human_position?x=${x}&distance=${distance}`)
                .then(response => response.json())
                .then(data => {
                    // Optional: Update status only for significant changes to avoid flooding
                    // statusDiv.innerHTML = `Position: x=${x}, distance=${distance}`;
                })
                .catch(error => console.error('Error updating position:', error));
        }

        // Control button handlers
        document.getElementById('startTracking').addEventListener('click', () => {
            fetch('/control/tracking')
                .then(response => response.json())
                .then(data => {
                    currentModeSpan.textContent = `Mode: ${data.mode}`;
                    statusDiv.innerHTML += `<br>Command sent: Start tracking`;
                });
        });
        
        document.getElementById('stopTracking').addEventListener('click', () => {
            fetch('/control/stop')
                .then(response => response.json())
                .then(data => {
                    currentModeSpan.textContent = `Mode: ${data.mode}`;
                    statusDiv.innerHTML += `<br>Command sent: Stop robot`;
                });
        });
        document.getElementById('startSearch').addEventListener('click', () => {
            fetch('/control/search')
                .then(response => response.json())
                .then(data => {
                    currentModeSpan.textContent = `Mode: ${data.mode}`;
                    statusDiv.innerHTML += `<br>Command sent: Start search`;
                });
        });

        // Create safety circle points
        function generateSafetyCircle() {
            console.log("Generating safety circle");
            const angles = [];
            const distances = [];
            for (let angle = 0; angle <= 360; angle += 5) {
                console.log(`Angle: ${angle}`);
                angles.push(angle);
                distances.push(1);
            }
            return {angles, distances};
        }
        
        const safetyCircle = generateSafetyCircle();

        // Create initial empty plot
        const plot = Plotly.newPlot(plotDiv, [
            {
                type: 'scatterpolar',
                r: [],
                theta: [],
                mode: 'markers',
                name: 'Scan Points',
                marker: { size: 3, color: 'rgba(70, 130, 180, 0.5)' }
            },
            {
                type: 'scatterpolar',
                r: [],
                theta: [],
                mode: 'markers',
                name: 'Obstacles',
                marker: { size: 12, color: 'red', symbol: 'x' }
            },
            {
                type: 'scatterpolar',
                r: safetyCircle.distances,
                theta: safetyCircle.angles,
                mode: 'lines',
                name: 'Safety Zone',
                line: { color: 'rgba(255, 0, 0, 0.7)', width: 1, dash: 'dash' },
                hoverinfo: 'none'
            }
        ], {
            polar: {
                radialaxis: { range: [0, 5] },
                angularaxis: { direction: "clockwise", rotation: 90 }
            },
            showlegend: true
        });



        function updatePlot() {
    fetch('/data')
        .then(response => response.json())
        .then(data => {
            // Update status with more details
            statusDiv.innerHTML = `
                <strong>Cycle:</strong> ${data.cycle_count} | 
                <strong>Obstacles:</strong> ${data.obstacles.length} | 
                <strong>Points:</strong> ${data.distances.length}
            `;
            
            // Process obstacles
            const obstacleAngles = data.obstacles.map(obs => 
                (Math.atan2(obs.y, obs.x) * 180/Math.PI + 360) % 360
            );
            const obstacleDistances = data.obstacles.map(obs => 
                Math.sqrt(obs.x*obs.x + obs.y*obs.y)
            );
            
            // Create obstacle hover text with tracking info
            const hoverTexts = data.obstacles.map(obs => {
                const speed = obs.speed !== undefined ? obs.speed.toFixed(2) : 'N/A';
                return `ID: ${obs.id || 'N/A'}<br>` + 
                       `Distance: ${obs.distance.toFixed(2)}m<br>` +
                       `Speed: ${speed} m/s<br>` + 
                       `Position: (${obs.x.toFixed(2)}, ${obs.y.toFixed(2)})`;
            });
            
            // Create velocity vectors for obstacles
            const arrowTraces = [];
            data.obstacles.forEach(obs => {
                if (obs.vx !== undefined && obs.vy !== undefined && (obs.vx !== 0 || obs.vy !== 0)) {
                    // Start point (obstacle position)
                    const r0 = Math.sqrt(obs.x*obs.x + obs.y*obs.y);
                    const theta0 = (Math.atan2(obs.y, obs.x) * 180/Math.PI + 360) % 360;
                    
                    // End point (velocity vector)
                    // Scale factor adjusts vector length for visibility
                    const scale = 1.0; 
                    const endX = obs.x + obs.vx * scale;
                    const endY = obs.y + obs.vy * scale;
                    const r1 = Math.sqrt(endX*endX + endY*endY);
                    const theta1 = (Math.atan2(endY, endX) * 180/Math.PI + 360) % 360;
                    
                    arrowTraces.push({
                        type: 'scatterpolar',
                        r: [r0, r1],
                        theta: [theta0, theta1],
                        mode: 'lines',
                        line: {
                            color: 'rgba(255, 165, 0, 0.8)',
                            width: 2
                        },
                        showlegend: false,
                        hoverinfo: 'none'
                    });
                }
            });

            // Determine obstacle marker colors based on speed
            const markerColors = data.obstacles.map(obs => {
                if (!obs.speed) return 'red';  // Default red for stationary
                
                // Color scale from green (slow) to red (fast) 
                const maxSpeed = 1.0;  // Adjust based on your expected speeds
                const speedRatio = Math.min(obs.speed / maxSpeed, 1);
                
                // Calculate RGB for a green-yellow-red gradient
                const r = Math.round(255 * speedRatio);
                const g = Math.round(255 * (1 - speedRatio));
                return `rgb(${r}, ${g}, 0)`;
            });

            // Create plots - include raw data, obstacles with velocity, and safety circle
            const plotData = [
                // Raw scan points
                {
                    type: 'scatterpolar',
                    r: data.distances,
                    theta: data.angles,
                    mode: 'markers',
                    name: 'Scan Points',
                    marker: { size: 3, color: 'rgba(70, 130, 180, 0.5)' }
                },
                // Obstacles with tracking data
                {
                    type: 'scatterpolar',
                    r: obstacleDistances,
                    theta: obstacleAngles,
                    mode: 'markers+text',
                    name: 'Tracked Obstacles',
                    text: data.obstacles.map(obs => obs.id),
                    textposition: 'top right',
                    textfont: {
                        family: 'sans-serif',
                        size: 12,
                        color: '#000000'
                    },
                    marker: { 
                        size: 12, 
                        color: markerColors,
                        symbol: 'circle' 
                    },
                    hovertext: hoverTexts,
                    hoverinfo: 'text'
                },
                // Safety zone
                {
                    type: 'scatterpolar',
                    r: safetyCircle.distances,
                    theta: safetyCircle.angles,
                    mode: 'lines',
                    name: 'Safety Zone',
                    line: { color: 'rgba(255, 0, 0, 0.7)', width: 1, dash: 'dash' },
                    hoverinfo: 'none'
                }
            ];

            // Add velocity vector arrows to plot data
            plotData.push(...arrowTraces);

                        if (data.human) {
                const humanAngle = data.human.angle;
                const humanDistance = data.human.distance;
                
                // Add human marker to plotData
                plotData.push({
                    type: 'scatterpolar',
                    r: [humanDistance],
                    theta: [humanAngle],
                    mode: 'markers+text',
                    name: 'Human',
                    text: ['👤'],
                    textposition: 'top center',
                    textfont: {
                        family: 'sans-serif',
                        size: 18,
                        color: '#000000'
                    },
                    marker: { 
                        size: 15, 
                        color: 'blue',
                        symbol: 'circle',
                        line: {
                            color: 'white',
                            width: 2
                        }
                    },
                    hoverinfo: 'text',
                    hovertext: `Human<br>Distance: ${humanDistance.toFixed(2)}m<br>Angle: ${humanAngle.toFixed(1)}°`,
                    showlegend: true
                });
                
                // Add a line from origin to human (optional)
                plotData.push({
                    type: 'scatterpolar',
                    r: [0, humanDistance],
                    theta: [0, humanAngle],
                    mode: 'lines',
                    line: {
                        color: 'rgba(0, 0, 255, 0.5)',
                        width: 2,
                        dash: 'dot'
                    },
                    showlegend: false,
                    hoverinfo: 'none'
                });
            }

            if (data.human) {
                // Add a distance ring around human's position
                const ringPoints = 36;
                const ringRadius = 0.5; // 0.5 meter radius around human
                const ringAngles = [];
                const ringDistances = [];
                
                for (let i = 0; i <= ringPoints; i++) {
                    const angle = (i / ringPoints) * 360;
                    ringAngles.push(angle);
                    ringDistances.push(data.human.distance);
                }
                
                plotData.push({
                    type: 'scatterpolar',
                    r: ringDistances,
                    theta: ringAngles,
                    mode: 'lines',
                    line: {
                        color: 'rgba(0, 0, 255, 0.3)',
                        width: 1
                    },
                    showlegend: false,
                    hoverinfo: 'none'
                });
            }

            
            // Update the plot
            Plotly.react(plotDiv, plotData, {
                polar: {
                    radialaxis: { range: [0, 5] },
                    angularaxis: { direction: "clockwise", rotation: 90 }
                },
                showlegend: true
            });
        })
        .catch(error => console.error('Error:', error));
}

        // Update every 250ms
        setInterval(updatePlot, 250);
        updatePlot(); // Initial update
    </script>
</body>
</html>
"""

def log(message):
    if VERBOSE:
        log_info("FLASK", message)

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

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

# Initialize when module is imported
initialize()



if __name__ == '__main__':
    # Start Flask app directly if this script is executed
    log_info("FLASK", "Starting Flask server...")
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)