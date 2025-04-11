from flask import Flask, render_template_string, jsonify
import zmq
from threading import Thread, Lock
import time
import json
import numpy as np
import logging
from utils.colored_logger import log_info, log_error, log_debug

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
    </style>
</head>
<body>
    <h1>LIDAR Real-Time Visualization</h1>
    <div id="status">Status: Waiting for initial data...</div>
    <div id="plot"></div>

    <script>
        // Initialize plot
        const plotDiv = document.getElementById('plot');
        const statusDiv = document.getElementById('status');
        
        // Create safety circle points
        function generateSafetyCircle() {
            console.log("Generating safety circle");
            const angles = [];
            const distances = [];
            for (let angle = 0; angle <= 360; angle += 5) {
                console.log(`Angle: ${angle}`);
                angles.push(angle);
                distances.push(1.5);
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

# Create a function to initialize everything
def initialize():
    # Start ZMQ listener thread
    zmq_thread = Thread(target=zmq_listener)
    zmq_thread.daemon = True
    zmq_thread.start()
    log_info("FLASK", "ZMQ listener started")

# Initialize when module is imported
initialize()

if __name__ == '__main__':
    # Start Flask app directly if this script is executed
    log_info("FLASK", "Starting Flask server...")
    app.run(host='0.0.0.0', port=5000, debug=False, use_reloader=False)