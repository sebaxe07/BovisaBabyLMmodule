# Raspberry Pi Distributed System Architecture

## Introduction

The Bovisa Baby Controller System employs a distributed computing architecture utilizing two Raspberry Pi devices working in tandem. This dual-Pi approach creates a specialized division of computational tasks, optimizing performance by allocating resources based on processing requirements.

The two-Pi architecture is structured as follows:

- **Raspberry Pi 4**: Functions as the primary control unit managing:

  - Central system coordination through the main controller
  - LIDAR sensor data processing for obstacle detection
  - GPS data handling and geofence management
  - Motor control via Arduino interfaces
  - Web server hosting for the visualization interface

- **Raspberry Pi 5**: Dedicated to vision processing tasks requiring higher computational power:
  - Camera input processing
  - Human detection and pose tracking using YOLOv8
  - April Tag detection for autonomous charging
  - Video stream processing and transmission

These devices communicate through ZMQ (ZeroMQ) message passing on dedicated ports:

- Port 5555: LIDAR data
- Port 5556: UI commands
- Port 5557: Camera commands
- Port 5558: Camera tracking data
- Port 5559: Camera video stream
- Port 5560: GPS data
- Port 5561: General communication

This distributed architecture provides several advantages:

1. **Task specialization**: Each Pi focuses on specific processing tasks
2. **Parallel processing**: Vision tasks run concurrently with navigation control
3. **Resource optimization**: Computing power allocated where most needed
4. **System reliability**: Failure in one component doesn't compromise the entire system

## Main Controller System

The Main Controller system forms the brain of the Bovisa Baby robot, orchestrating all robot operations by coordinating between various subsystems. Implemented in the `MainController` class within `controllers/main_controller.py`, this module handles state management, communication, and decision-making processes.

### System Architecture

The Main Controller adopts a centralized coordination approach with distributed execution:

1. **Core Controller**: Located on the primary Raspberry Pi 4, manages system state and high-level decision making
2. **Subsystem Processors**: Dedicated modules for specialized functions (LIDAR, GPS, camera tracking)
3. **Arduino Interface Layer**: Bridge between software commands and hardware execution

```python
class MainController:
    def __init__(self, config):
        log_info("CONTROLLER", "Initializing main controller")
        self.config = config

        # Initialize all Arduino interfaces
        self.arduino_manager = ArduinoManager(config)
        # For backward compatibility
        self.arduino = self.arduino_manager.motor_arduino

        self._setup_communication()
        self._setup_lidar()
        self._setup_gps()
        # Camera is now on a different device, no need to set it up locally
```

### Communication Infrastructure

The Main Controller establishes a robust communication network using ZeroMQ (ZMQ), implementing:

1. **Publisher-Subscriber Pattern**: Enables efficient one-to-many message distribution
2. **Request-Reply Pattern**: Used for command execution and responses
3. **Cross-Device Communication**: Enables seamless data exchange between the two Raspberry Pi units

```python
def _setup_communication(self):
    context = zmq.Context()
    # Subscribe to LIDAR data
    self.subscriber = context.socket(zmq.SUB)
    self.subscriber.connect(f"tcp://localhost:{self.config['lidar']['communication']['port']}")
    self.subscriber.setsockopt_string(zmq.SUBSCRIBE, '')

    # Add subscription for camera tracking data
    self.camera_subscriber = context.socket(zmq.SUB)
    self.camera_subscriber.connect(f"tcp://{self.config['camera']['communication']['ip']}:{self.config['camera']['communication']['tracking_port']}")
    self.camera_subscriber.setsockopt_string(zmq.SUBSCRIBE, '')

    # Create publisher for camera commands
    self.camera_command_publisher = context.socket(zmq.PUB)
    self.camera_command_publisher.bind(f"tcp://{self.config['controller']['communication']['ip']}:{self.config['camera']['communication']['command_port']}")
```

### State Management System

The controller implements a sophisticated state machine that governs robot behavior:

1. **IDLE**: Default state, robot awaits commands
2. **SEARCH**: Actively looking for humans to track
3. **TRACKING**: Following detected humans with appropriate distance maintenance
4. **AVOIDING**: Temporarily navigating around obstacles while maintaining tracking goal
5. **CHARGING**: Seeking and approaching charging station using AprilTag detection

State transitions are triggered by:

- UI commands from the web interface
- Sensor events (obstacle detection, human found/lost)
- System conditions (battery level, geofence status)

### Arduino Interface for Movement Control

The Main Controller communicates with multiple Arduino boards through a dedicated interface layer:

1. **Primary Motor Arduino**: Controls main drive motors for movement
2. **Sensor Arduino**: Handles additional sensors and feedback systems
3. **Power Management Arduino**: Monitors battery status and charging

```python
def send_command(self, command):
    """Send a command to the Arduino for robot movement"""
    if self.mock_mode:
        self._process_mock_command(command)
        log_debug("Arduino", f"Mock command sent: {command}")
        return

    # For numerical commands (position values), use analog control
    if isinstance(command, (int, float)):
        # Map from -10,10 to motor control values
        position = float(command)

        # Send the analog position command in binary format
        command_code = 0x02  # Analog position command
        try:
            # Pack as: command_code (byte) + position (float)
            packed_data = struct.pack('<Bf', command_code, position)
            self.serial.write(packed_data)
            self.serial.flush()
            log_debug("Arduino", f"Sent analog position command: {position:.2f}")
        except Exception as e:
            log_error("Arduino", f"Failed to send analog position: {e}")
    else:
        # String command handling (stop, forward, etc.)
        command_map = {
            "stop": 0x00,
            "forward": 0x01,
            "backward": 0x02,
            "left": 0x03,
            "right": 0x04,
            "dock": 0x05   # Special command for docking
        }
```

### Reactive Processing Loop

The controller continuously processes incoming messages from all subsystems:

1. **Message Prioritization**: GPS and geofence data take highest priority
2. **Command Processing**: UI commands are evaluated and executed
3. **Sensor Integration**: Data from LIDAR, GPS, and camera systems are integrated
4. **Decision Execution**: Commands are sent to Arduino for physical execution

```python
def process_messages(self):
    while True:
        # Check for GPS data (highest priority)
        try:
            gps_msg = self.gps_subscriber.recv_json(zmq.NOBLOCK)
            # Process GPS data and geofence status
        except zmq.Again:
            pass

        # Check for camera tracking data
        try:
            camera_msg = self.camera_subscriber.recv_json(zmq.NOBLOCK)
            # Process human tracking information
        except zmq.Again:
            pass

        # Check for LIDAR obstacle data
        try:
            lidar_msg = self.subscriber.recv_json(zmq.NOBLOCK)
            # Process obstacle avoidance data
        except zmq.Again:
            pass

        # Check for UI commands
        try:
            cmd = self.command_subscriber.recv_json(zmq.NOBLOCK)
            # Process user interface commands
        except zmq.Again:
            pass
```

### Obstacle Avoidance Integration

The Main Controller implements a sophisticated obstacle avoidance strategy that:

1. **Preserves tracking goals** while navigating around obstacles
2. **Uses analog steering values** (-10 to 10) for smooth movement
3. **Prioritizes obstacles** based on distance and angular position
4. **Implements emergency stop** for critical situations

```python
def _avoidance_strategy(self, obstacles):
    """
    Advanced obstacle avoidance using analog position values (-10 to 10)
    to provide smooth, intelligent turning behavior.

    The algorithm works as follows:
    1. Direction determination:
       - If obstacle is on the left side, turn right (negative value)
       - If obstacle is on the right side, turn left (positive value)

    2. Urgency calculation:
       - Center obstacles generate higher urgency (sharper turns)
       - Edge obstacles generate lower urgency (gentler turns)
       - Closer obstacles increase urgency

    3. Emergency stop:
       - Very close obstacles directly in front trigger emergency stop
    """
    # Implementation details...
```

### Geofence Management

The controller actively enforces geofence boundaries by:

1. **Monitoring GPS position** relative to defined boundaries
2. **Detecting boundary violations** and triggering response
3. **Implementing return-to-safety behavior** when outside boundaries
4. **Resuming normal operation** once back within safe zone

This safety feature ensures the robot remains within designated operational areas.

### System Integration and Communications

![Controller Architecture](https://i.imgur.com/diagram-placeholder.png)

The Main Controller serves as the integration hub for all robot subsystems:

1. **Cross-Module Coordination**: Ensures all subsystems work together coherently
2. **Consistent State Management**: Maintains system stability during transitions
3. **Error Handling**: Implements graceful degradation and recovery strategies
4. **Configuration Management**: Loads and applies settings from configuration files

The use of ZeroMQ for inter-process and inter-device communication provides:

- **Low latency**: Critical for real-time control systems
- **Reliability**: Message delivery guarantees
- **Scalability**: Easy addition of new subsystems
- **Flexibility**: Support for various communication patterns

### Core System Architecture Summary

The Main Controller represents the cornerstone of the Bovisa Baby system, providing the intelligence and coordination necessary for autonomous operation. By effectively managing the information flow between sensors, processors, and actuators, it enables complex behaviors like human tracking, obstacle avoidance, and autonomous charging while maintaining safety constraints such as geofencing.

## Subsystem Implementation Details

Having covered the central coordination system, we will now explore each of the specialized subsystems in depth. Each component plays a critical role in the robot's operation, from spatial awareness to human interaction capabilities.

## Tasks Completed

### GPS System and Geofencing Implementation

Our team implemented a comprehensive GPS and geofencing system that allows the robot to maintain awareness of its geographic position and operate within defined boundaries. This system consists of several key components:

#### GPS Processing Module

The `GpsProcessor` class in `sensors/gps_processor.py` handles all GPS data processing. It can operate in two modes:

1. **Real GPS Mode**: Interfaces with a NEO-M8N GPS module connected via serial port, parsing NMEA sentences to extract position, altitude, speed, and satellite information.
2. **Mock GPS Mode**: Generates simulated GPS data for testing purposes, especially useful when developing indoors.

The system supports key GPS features:

- Position tracking (latitude/longitude)
- Altitude, speed, and course monitoring
- Satellite tracking and signal quality assessment (HDOP)
- Fix quality monitoring

```python
# GPS configuration in settings.yaml
gps:
  mock_mode: false   # Set to false when real GPS module is connected
  port: "/dev/ttyAMA0"  # Serial port for GPS module
  baud_rate: 38400      # Default baud rate for NEO-M8N
  update_rate: 1.0      # Update frequency in seconds
```

#### Geofence System

The geofencing capability allows defining geographic boundaries within which the robot must operate. When the robot approaches or crosses these boundaries, appropriate alerts are generated and automatic return behavior is triggered.

Key geofence features:

- **Polygon-based boundaries**: Defined in JSON files (e.g., `geofenceBovisa.json`)
- **Distance calculations**: Real-time monitoring of distance to geofence edges
- **Warning thresholds**: Early warnings when approaching boundaries
- **Return-to-geofence behavior**: Automatic navigation back to permitted areas

Geofence configuration:

```yaml
# Geofence settings in settings.yaml
geofence:
  enabled: true
  geofence_file: "config/geofenceBovisa.json" # Path to boundary definition
  warning_distance: 10 # Warning distance in meters
```

The geofence boundaries are defined in GeoJSON format:

```json
{
  "type": "Polygon",
  "name": "Bovisa Campus Boundary",
  "coordinates": [
    [9.1659558, 45.5063374],
    [9.1661221, 45.5059992],
    ...
  ],
  "properties": {
    "color": "#ff0000",
    "fillColor": "#ff6666"
  }
}
```

#### Geofence Integration with Control System

The geofence system is fully integrated with the main controller, which takes appropriate actions when geofence alerts are received:

1. **Warning Level**: When approaching the boundary (within warning_distance), the system logs warnings but continues operation.
2. **Critical Level**: When outside the boundary, the system:
   - Stores the current operating state
   - Switches to "RETURNING" mode
   - Calculates bearing to geofence center
   - Commands the robot to move in that direction
   - Restores previous state once safely inside the boundary

Example from `main_controller.py`:

```python
def _process_geofence_alert(self, alert):
    if alert_level == 'critical':
        log_error("CONTROLLER", "CRITICAL GEOFENCE ALERT - Outside permitted area!")
        self.previous_state = self.current_state
        self.current_state = "RETURNING"
        self.returning_to_geofence = True
        self.return_bearing = direction
```

#### Web Interface Visualization

The geofence system includes visual feedback through the web interface:

- Real-time map showing robot position relative to geofence
- Color-coded status indicators (inside/outside)
- Distance to boundary display
- Visual representation of the geofence polygon

This implementation ensures the robot operates within its designated area and automatically returns when it strays beyond boundaries, providing both safety and operational reliability.

### LIDAR-Based Obstacle Detection and Avoidance

Our team implemented a sophisticated LIDAR-based obstacle detection and avoidance system that enables the robot to navigate safely through complex environments while avoiding collisions. This system consists of several key components:

#### LIDAR Processor Module

The `LidarProcessor` class in `sensors/lidar_processor.py` provides real-time obstacle detection using RPLIDAR data. It features:

1. **Dual-mode operation**:

   - **Real LIDAR mode**: Directly interfaces with RPLIDAR hardware through PyRPlidar
   - **Mock mode**: Generates simulated LIDAR data for testing without hardware

2. **Advanced obstacle detection**:
   - Converts polar scan data (angle/distance) to Cartesian coordinates
   - Filters data points based on configurable safety distances
   - Implements sophisticated clustering using DBSCAN algorithm
   - Tracks obstacles between scans with unique IDs and velocity estimation

```python
# LIDAR configuration in settings.yaml
lidar:
  mock_mode: false
  port: "/dev/ttyLIDAR"
  safety_distance: 1 # meters
  update_rate: 10 # Hz
  scan_interval: 0.1 # seconds
  # Avoidance parameters
  front_detection_angle: 90 # degrees (±45° from center)
  angular_urgency_weight: 0.7 # How much weight to give to angular position
  distance_urgency_weight: 0.3 # How much weight to give to distance
  # Emergency stop parameters
  emergency_stop_distance: 0.3 # meters - immediate stop threshold
  emergency_stop_angle: 15 # degrees - center angle for emergency stop
```

#### Obstacle Avoidance System

The system implements a dynamic avoidance strategy with continuous analog steering values instead of discrete state-based turning. This approach provides several advantages:

1. **Proportional response**:

   - Obstacles directly ahead cause sharp turns (higher values)
   - Obstacles at the edges cause gentle turns (lower values)
   - Turn magnitude scales with proximity (closer obstacles = stronger response)

2. **Directional intelligence**:

   - Obstacles on the left trigger right turns (negative values)
   - Obstacles on the right trigger left turns (positive values)

3. **Emergency stop capability**:
   - Very close obstacles directly ahead trigger immediate stops
   - System differentiates between obstacles and tracked humans

Key implementation from `main_controller.py`:

```python
def _avoidance_strategy(self, obstacles):
    """
    Advanced obstacle avoidance using analog position values (-10 to 10)
    to provide smooth, intelligent turning behavior.
    """
    # Get the front detection cone angle from config
    front_cone_angle = self.config['lidar']['front_detection_angle']

    # Find closest obstacle in front cone
    closest_obstacle = min(front_obstacles, key=lambda o: o['distance'])

    # Emergency stop check
    emergency_stop_distance = self.config['lidar']['emergency_stop_distance']
    emergency_stop_angle = self.config['lidar']['emergency_stop_angle']
    if closest_obstacle['distance'] < emergency_stop_distance and abs(obstacle_angle) < emergency_stop_angle:
        return None  # Special return value to indicate emergency stop

    # Calculate turn direction and magnitude
    turn_direction = -1.0 if obstacle_angle > 0 else 1.0
    turn_magnitude = 10.0 - (angle_ratio * (10.0 - min_turn_value))
    avoidance_value = turn_direction * turn_magnitude

    return avoidance_value
```

#### Human-Obstacle Differentiation

The system intelligently distinguishes between obstacles and the human being tracked:

1. When in TRACKING mode, the system compares LIDAR obstacle positions with the camera-tracked human position
2. This prevents the robot from avoiding the human it's supposed to follow
3. Position and distance thresholds determine which LIDAR points to ignore
4. The remaining obstacles are processed for avoidance

#### LIDAR Quality Management

The system includes robust mechanisms for ensuring reliable LIDAR operation:

1. **Scan quality monitoring**:

   - Tracks the number of points per scan to detect quality degradation
   - Automatically attempts recovery when quality falls below thresholds

2. **Error recovery**:
   - Graceful disconnection and reconnection on errors
   - Progressive waiting periods to prevent rapid cycling

This LIDAR-based detection and avoidance system enables the robot to navigate dynamically through its environment, detecting and avoiding obstacles while maintaining its primary tasks like human tracking.

### Camera-Based Human Detection and Tracking System

Our team implemented a comprehensive camera-based vision system for human detection, tracking, and autonomous charging. This advanced vision system operates on the Raspberry Pi 5, leveraging its enhanced computational capabilities for real-time processing of video feeds.

#### Human Detection and Pose Recognition

The system utilizes YOLOv8 in pose estimation mode to detect humans and their key body points:

1. **Advanced Model Implementation**:

   - Uses YOLOv8 pose model optimized for Raspberry Pi 5
   - Pre-optimized as ONNX for accelerated inference
   - Multi-threaded implementation using all CPU cores

2. **Keypoint-Based Detection**:

   - Tracks 17 anatomical keypoints for each detected person
   - Includes nose, eyes, shoulders, elbows, wrists, hips, knees, and ankles
   - Maintains confidence values for each keypoint to assess reliability

3. **Robust Tracking System**:
   - Integrates ByteTrack for reliable multi-person tracking
   - Assigns unique IDs to maintain identity through occlusions
   - Enables smooth tracking even during brief disappearances

```python
# Configuration settings for camera processing
camera:
  port: "/dev/video0"
  tolerance: 1 # meters
  close_distance: 1 # meters
  charging_close_distance: 0.5 # meters
  focal_length: 406.19
  known_height: 1.94
  model: "yolov8n-pose.onnx"
  max_age: 15
  confidence_threshold: 0.5
  debug_visualization: true
  target_fps: 20
```

#### Distance Estimation and Calibration

The system employs multiple methods to accurately estimate the distance to detected humans:

1. **Geometric Distance Estimation**:

   - Uses the inverse relationship between apparent size and distance
   - Formula: `distance = (known_height * focal_length) / apparent_height`
   - Calibrated through a dedicated calibration tool (`tools/calibrate_distance.py`)

2. **Keypoint-Based Improvements**:

   - Accounts for partially visible humans by analyzing visible keypoints
   - Applies anatomical proportions to estimate full height
   - Compensates for people at the edge of the frame

3. **Smart Filtering**:
   - Implements median filtering to reduce distance measurement noise
   - Maintains a history of recent measurements for stability
   - Adapts to movement with weighted recent samples

```python
def estimate_distance_from_keypoints(self, keypoints, box_height, y1, y2, frame_height):
    # Extract key anatomical points with confidence above threshold
    # Shoulders, hips, ankles tracking for improved estimation
    if has_shoulders and has_hips:
        # Calculate distance using the torso height
        # More reliable than full body when legs are occluded
        shoulder_hip_height = self._calculate_vertical_distance(shoulders_mid, hips_mid)
        # Torso is approximately 40% of full body height
        estimated_full_height = shoulder_hip_height / 0.4
        distance = (self.known_height * self.focal_length) / estimated_full_height
    elif has_shoulders and has_head:
        # Calculate using upper body proportions
        # Head-to-shoulders is approximately 22% of height
        head_shoulder_height = self._calculate_vertical_distance(nose, shoulders_mid)
        estimated_full_height = head_shoulder_height / 0.22
        distance = (self.known_height * self.focal_length) / estimated_full_height
```

#### Human Position Tracking

The system provides precise positional data to enable the robot to follow humans:

1. **Normalized Position Coordinates**:

   - Maps detected human positions to a -10 to 10 scale
   - Center of frame is 0, left edge is -10, right edge is +10
   - Enables intuitive directional control commands

2. **Tracking Persistence**:

   - Maintains "locked-on" tracking of a selected individual
   - Avoids tracking confusion in multi-person scenarios
   - Provides status updates (TRACKING, NOTFOUND, LOST)

3. **Target Selection Logic**:
   - Initially selects the closest person as primary target
   - Maintains tracking through object ID persistence
   - Implements priority filtering for reliable tracking

#### AprilTag Detection for Autonomous Charging

Our system implements AprilTag detection for precise docking with charging stations:

1. **Tag Detection**:

   - Uses the pupil_apriltags library for efficient tag detection
   - Supports multiple tag families (tag25h9 employed)
   - Provides sub-pixel accuracy for tag corners and centers

2. **Charging Behaviors**:

   - Different tags (IDs 0-2) trigger different alignment behaviors
   - Center alignment (ID 0): Robot centers itself on the tag
   - Left alignment (ID 1): Robot positions tag on its left side
   - Right alignment (ID 2): Robot positions tag on its right side

3. **Precision Docking**:
   - Calculates precise distance to tags using known physical size
   - Enables smooth approach with decreasing speed
   - Provides terminal guidance for charging station alignment

```python
# In charging mode, processing tag information
if tags:
    for tag in tags:
        tag_id = tag.tag_id
        center = tag.center
        corners = tag.corners

        # Calculate the width of the tag in pixels
        width1 = np.linalg.norm(corners[1] - corners[0])  # Top edge
        width2 = np.linalg.norm(corners[2] - corners[3])  # Bottom edge
        tag_width_px = (width1 + width2) / 2

        # Calculate distance based on known tag size
        distance = (self.tag_size * self.focal_length) / tag_width_px
```

#### System Integration

The camera system integrates with the main robot architecture through several mechanisms:

1. **ZMQ Communication**:

   - Publishes tracking data on dedicated ZMQ channels
   - Streams video feed for remote visualization
   - Receives commands for mode switching (tracking/charging)

2. **Cross-Device Synchronization**:

   - Camera processing runs on Raspberry Pi 5
   - Results transmitted to Raspberry Pi 4 control system
   - Maintains consistent frame rates despite processing demands

3. **Resource Management**:
   - Implements dynamic FPS control based on current task
   - Monitors and reports CPU temperature
   - Gracefully handles resource constraints

This sophisticated vision system enables the robot to reliably detect, track, and follow humans through complex environments while also supporting precise charging station docking using AprilTag detection.

### Web-Based Control and Visualization System

Our team developed a comprehensive web-based interface for robot control, monitoring, and visualization. Implemented using Flask and modern web technologies, this system provides an intuitive dashboard for robot operation and real-time feedback.

#### Architecture and Components

The visualization system consists of three main components:

1. **Flask Web Server**:

   - Lightweight Python web server handling HTTP requests
   - RESTful API endpoints for commands and data
   - WebSocket-like connections for real-time updates

2. **ZeroMQ Communication Bridge**:

   - Connects the web interface to robot subsystems
   - Relays commands from user interface to control system
   - Streams sensor data to web clients in real time

3. **Interactive Frontend**:
   - Dynamic visualizations using Plotly.js and Leaflet
   - Responsive control panels for robot operations
   - Real-time data displays with automatic updates

```python
# Server-side ZMQ connection for command publishing
def setup_command_publisher():
    global command_publisher
    context = zmq.Context()
    command_publisher = context.socket(zmq.PUB)
    command_publisher.bind("tcp://*:5556")  # Command channel
    log_info("FLASK", "Command publisher initialized on port 5556")
```

#### Real-Time Visualizations

The system provides multiple synchronized visualizations:

1. **LIDAR Visualization**:

   - Polar plot showing real-time LIDAR scan data
   - Color-coded obstacle highlighting with distance indicators
   - Safety distance rings and detection cone visualization
   - Dynamic updates with obstacle tracking

2. **GPS and Geofencing**:

   - Interactive map showing robot position in real-time
   - Geofence boundary visualization with color-coded status
   - Distance-to-edge indicators and warning displays
   - Map follow/free mode for user exploration

3. **Camera Feed**:
   - Live streaming of processed camera feed
   - Visualization overlays showing detected humans and tracking status
   - AprilTag detection visualization during charging operations
   - Frame rate optimization for network efficiency

#### Control Interface

The web system provides a comprehensive set of controls for robot operation:

1. **Operation Modes**:

   - **Start Tracking**: Activates human tracking mode
   - **Stop Robot**: Immediately halts all movement
   - **Start Search**: Initiates active human searching
   - **Start Charging**: Begins autonomous charging station location

2. **Status Monitoring**:

   - Current operation mode display
   - System status indicators
   - Battery level monitoring
   - Error state visualization

3. **Manual Control**:
   - Direction controls for manual operation
   - Speed adjustment sliders
   - Emergency stop button
   - System reset capabilities

```javascript
// Client-side control handling
document.getElementById("startTracking").addEventListener("click", function () {
  fetch("/control/tracking")
    .then((response) => response.json())
    .then((data) => {
      currentModeSpan.textContent = `Mode: ${data.mode}`;
      updateStatus(`Changed mode to ${data.mode}`);
    });
});
```

#### Responsive Design

The user interface employs responsive design principles for accessibility across devices:

1. **Multi-device Support**:

   - Desktop browsers for full-featured operation
   - Tablet-optimized layout for field use
   - Mobile view for basic monitoring and control

2. **Layout Adaptation**:
   - Flexible grid system using CSS Grid and Flexbox
   - Component reflow based on screen size
   - Touch-friendly controls for mobile operation

#### Implementation Details

The web system is implemented using modern web technologies:

1. **Server-Side**:

   - Flask framework for HTTP handling and template rendering
   - ZeroMQ for asynchronous messaging with robot subsystems
   - JSON data formatting for efficient communication
   - Thread-safe data structures for concurrent access

2. **Client-Side**:
   - Vanilla JavaScript for lightweight performance
   - Plotly.js for interactive charts and visualizations
   - Leaflet for map integration and geospatial visualization
   - CSS3 for styling and responsive design

```html
<!-- Dashboard UI structure -->
<div class="dashboard-container">
  <div class="column">
    <!-- Control Panel -->
    <div id="controls">
      <button id="startTracking" class="control-button start-button">
        Start Tracking
      </button>
      <button id="stopTracking" class="control-button stop-button">
        Stop Robot
      </button>
      <button id="startSearch" class="control-button sim-button">
        Start Search
      </button>
      <button id="startCharging" class="control-button charge-button">
        Start Charging
      </button>
      <span id="current-mode">Mode: IDLE</span>
    </div>

    <!-- Video Stream -->
    <div id="video-container">
      <h2>Camera Feed</h2>
      <img
        id="video-stream"
        src="{{ url_for('video_feed') }}"
        alt="Video stream"
      />
    </div>

    <!-- GPS Map -->
    <div id="gps-container">
      <h2>GPS Location</h2>
      <div id="gps-map"></div>
      <div id="gps-status">
        <div>Position: <span id="gps-position">Waiting for data...</span></div>
        <div>Geofence: <span id="geofence-status">Inside</span></div>
      </div>
    </div>
  </div>
</div>
```

#### Data Flow and Integration

The web system is fully integrated with the robot's distributed architecture:

1. **Command Flow**:

   - User inputs in web interface trigger RESTful API calls
   - Server processes commands and publishes to ZMQ channels
   - Main controller receives and executes commands
   - Status updates flow back to interface via ZMQ

2. **Data Flow**:

   - Sensors publish data to dedicated ZMQ channels
   - Web server subscribes to relevant channels
   - Server processes and formats data for web clients
   - Clients render visualizations with optimized update rates

3. **Cross-Device Communication**:
   - Web interface can communicate with both Raspberry Pi devices
   - Camera feed from Pi 5 streams through web server on Pi 4
   - Unified control interface despite distributed processing

#### Security Considerations

The system implements several security measures:

1. **Local Network Operation**: Interface designed for local network use only
2. **Input Validation**: All user inputs validated server-side before processing
3. **Command Rate Limiting**: Prevention of command flooding
4. **Graceful Error Handling**: Robust response to unexpected inputs

This comprehensive web interface provides an intuitive yet powerful means to control, monitor, and debug the robot system, enhancing both operator experience and development efficiency.

## Problems Encountered

During the development and implementation of the Bovisa Baby robot system, we faced several significant technical challenges that required innovative solutions:

### Computational Resource Constraints

The most significant challenge was the excessive computational demand when attempting to run all systems on a single Raspberry Pi:

1. **Vision Processing Bottleneck**:

   - Human detection and tracking algorithms required substantial CPU resources
   - Initial tests on a single Pi resulted in extremely low framerates (1-3 FPS)
   - Such low framerates made reliable human tracking nearly impossible

2. **Model Selection Challenges**:

   - We experimented with multiple models of varying sizes and complexities
   - Tested YOLOv5n, YOLOv5nu, YOLOv8n in various configurations (standard and quantized)
   - Created multiple ONNX optimized versions with different input resolutions (224px, 320px)
   - Each model represented different tradeoffs between accuracy and performance

3. **Solution: Dual Raspberry Pi Architecture**:
   - Dedicated the Raspberry Pi 5 exclusively to vision processing tasks
   - Took advantage of the Pi 5's improved CPU/GPU capabilities for neural network inference
   - Reserved the Pi 4 for LIDAR processing, GPS handling, and system coordination
   - This separation allowed us to achieve acceptable framerates (15-20 FPS) for human tracking

### Network Communication Issues

The inter-device communication presented unexpected challenges:

1. **Network Hardware Limitations**:

   - Initially planned to use an unmanaged network switch with Ethernet connections
   - The low-cost switch became overwhelmed by the high-volume ZMQ data streams
   - Resulted in packet loss, latency spikes, and intermittent connection failures
   - Communication quality degraded further during high message volume periods

2. **ZMQ Message Congestion**:

   - Video streaming and sensor data created high-bandwidth requirements
   - Message queues would occasionally overflow, causing data loss or stale information
   - Timing issues between subsystems emerged due to communication delays

3. **Solution: Wireless Hotspot Bridge**:
   - Utilized a smartphone as a dedicated WiFi hotspot
   - Created a private, isolated wireless network for inter-device communication
   - Implemented message throttling and prioritization
   - This approach proved more reliable than the budget switch solution

### Hardware Stability Problems

We encountered several hardware-related issues that complicated development:

1. **Raspberry Pi 4 USB Connectivity Failures**:

   - Intermittent and unpredictable USB port failures
   - Connected devices (LIDAR, Arduino) would suddenly become unresponsive
   - System logs showed no obvious error messages

2. **Recovery Procedure Development**:

   - Discovered a "software cycle" recovery method:
     1. Power off the Pi 4
     2. Remove the microSD card
     3. Power on without SD card
     4. Power off again
     5. Reinsert microSD and restart
   - This procedure restored USB functionality in most cases
   - Implemented automatic detection and notification for USB failures

3. **System Freezes**:
   - Random complete system freezes on the Pi 4
   - Required manual power cycling to recover

### Environmental Sensing Limitations

The robot's sensors exhibited limitations in certain environmental conditions:

1. **LIDAR Performance in Sunlight**:

   - Discovered severe performance degradation in direct sunlight
   - The RPLIDAR A1, designed primarily for indoor use, would return minimal or no data points
   - Detection range reduced by up to 90% in bright outdoor conditions
   - Obstacle avoidance became unreliable in these conditions

2. **Mitigation Approaches**:
   - Implemented adaptive thresholds based on point cloud density
   - Added heuristics to detect and report low-quality LIDAR data
   - Considered development of a protective shield/hood for the LIDAR sensor
   - Added code to fall back to more conservative movement when point cloud quality degraded

### Vision System Challenges

The monocular camera-based vision system presented inherent limitations:

1. **Distance Estimation Accuracy**:

   - Fundamental challenges in monocular depth estimation
   - Distance calculations based on apparent size were affected by:
     - Variations in human height
     - Partial visibility of the body
     - Changing postures and orientations

2. **Close Proximity Detection**:

   - Difficulty in accurately detecting when a human was "close enough" to the robot
   - Inconsistent triggering of proximity thresholds
   - Occasional failure to recognize when humans were within the designated "close distance"

3. **Improvement Strategies**:
   - Implemented anatomical proportions-based estimation
   - Added keypoint-specific distance calculations
   - Incorporated temporal smoothing and outlier rejection
   - Calibrated the system against known distances and heights

These challenges and their solutions highlight the complexity of developing a robust autonomous robot system. Each problem required careful analysis and creative engineering approaches to overcome.
