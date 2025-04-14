from flask import render_template, jsonify, request, Response
from utils.colored_logger import log_debug, log_info, log_error
from utils.flask_visualization.zmq_handlers import command_publisher
import time

def register_routes(app, scan_data, data_lock):
    """Register all Flask routes"""
    
    @app.route('/')
    def index():
        """Render the main visualization page"""
        return render_template('index.html')
    
    @app.route('/data')
    def get_data():
        """Return current scan data as JSON"""
        with data_lock:
            log_info("FLASK", f"Data requested - count: {scan_data['cycle_count']}, " + 
                            f"angles: {len(scan_data['angles'])}, " +
                            f"obstacles: {len(scan_data['obstacles'])}")
            
            # Add cache control headers to prevent browser caching
            response = jsonify(scan_data)
            response.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate'
            response.headers['Pragma'] = 'no-cache'
            response.headers['Expires'] = '0'
            return response
        
        
    @app.route('/video_feed')
    def video_feed():
        """Video streaming route"""
        return Response(generate_frames(scan_data, data_lock),
                        mimetype='multipart/x-mixed-replace; boundary=frame')
        
    @app.route('/debug')
    def debug_info():
        """Return debug information about the data state"""
        with data_lock:
            log_info("FLASK", "Debug route accessed")
            return jsonify({
                "scan_data_id": id(scan_data),  # Memory address of scan_data
                "cycle_count": scan_data["cycle_count"],
                "angles_count": len(scan_data["angles"]),
                "distances_count": len(scan_data["distances"]),
                "obstacles_count": len(scan_data["obstacles"]),
                "has_human": scan_data["human"] is not None,
                "timestamp": time.time()
            })

    @app.route('/control/tracking')
    def start_tracking():
        """Send tracking command to robot"""
        if command_publisher:
            command_publisher.send_json({
                'command': 'set_state',
                'state': 'TRACKING'
            })
            log_info("FLASK", "Sent command: Start tracking")
        return jsonify({'success': True, 'mode': 'TRACKING'})
    
    @app.route('/control/stop')
    def stop_robot():
        """Send stop command to robot"""
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
        """Send search command to robot"""
        if command_publisher:
            command_publisher.send_json({
                'command': 'set_state',
                'state': 'SEARCH'
            })
            log_info("FLASK", "Sent command: Start search")
        return jsonify({'success': True, 'mode': 'SEARCH'})
    
    @app.route('/control/update_human_position')
    def update_human_position():
        """Update mock human position"""
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

def generate_frames(scan_data, data_lock):
    """Generate video frames from scan_data"""
    while True:
        with data_lock:
            if scan_data.get("latest_frame") is not None:
                frame_data = scan_data["latest_frame"]
                
                # Create MJPEG frame format
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_data + b'\r\n')
                
        # Control the frame rate
        time.sleep(0.05)  # ~20 FPS