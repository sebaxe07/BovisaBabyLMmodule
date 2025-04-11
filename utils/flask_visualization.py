from flask import Flask, render_template_string, jsonify
import zmq
from threading import Thread, Lock
import time
import json
import numpy as np

app = Flask(__name__)

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



        // Corrected update function
        function updatePlot() {
            fetch('/data')
                .then(response => response.json())
                .then(data => {
                    // Update status
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
                    
                    // CORRECTED Plotly update syntax
                    Plotly.react(plotDiv, [
                        {
                            type: 'scatterpolar',
                            r: data.distances,
                            theta: data.angles,
                            mode: 'markers',
                            name: 'Scan Points',
                            marker: { size: 3, color: 'rgba(70, 130, 180, 0.5)' }
                        },
                        {
                            type: 'scatterpolar',
                            r: obstacleDistances,
                            theta: obstacleAngles,
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
                            line: { color: 'rgba(255, 0, 0, 0.7)', width: 1, dash: 'dash' }
                        }
                    ], {
                        polar: {
                            radialaxis: { range: [0, 5] },
                            angularaxis: { direction: "clockwise", rotation: 90 }
                        }
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

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/data')
def get_data():
    with data_lock:
        # Return current data with debug info
        print(f"Serving data: {len(scan_data['angles'])} points, {len(scan_data['obstacles'])} obstacles")
        return jsonify(scan_data)

def zmq_listener():
    context = zmq.Context()
    subscriber = context.socket(zmq.SUB)
    subscriber.connect("tcp://localhost:5555")
    subscriber.setsockopt_string(zmq.SUBSCRIBE, '')
    
    print("ZMQ listener started. Waiting for data...")
    
    while True:
        try:
            msg = subscriber.recv_json()
            print(f"Received message type: {msg['type']}")
            
            with data_lock:
                if msg['type'] == 'scan':
                    scan_data['angles'] = [float(a) for a in msg['data']['angles']]
                    scan_data['distances'] = [float(d) for d in msg['data']['distances']]
                    scan_data['cycle_count'] += 1
                    print(f"Updated scan data (Cycle {scan_data['cycle_count']})")
                    
                elif msg['type'] == 'obstacles':
                    scan_data['obstacles'] = msg['data']
                    print(f"Updated obstacles: {len(msg['data'])} found")
                
        except Exception as e:
            print(f"ZMQ error: {e}")
            time.sleep(0.1)

if __name__ == '__main__':
    # Start ZMQ listener thread
    zmq_thread = Thread(target=zmq_listener)
    zmq_thread.daemon = True
    zmq_thread.start()
    
    # Start Flask app
    print("\n----------------------------------------")
    print("LIDAR Visualization Server Running")
    print("1. First start your LIDAR processor")
    print("2. Then access the visualization at:")
    print("   http://localhost:5000")
    print("3. Check browser console (F12) for debug info")
    print("----------------------------------------\n")
    
    app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)