# Bovisa Baby Controller System

A modular robot control system with LIDAR-based obstacle detection, mock camera tracking, and motor control capabilities. The system provides a web-based visualization interface for real-time monitoring.

## Features
- LIDAR-based obstacle detection and tracking
- Camera-based human tracking (mock implementation)
- Robot movement control via Arduino interface
- Real-time visualization through web interface
- ZMQ-based inter-process communication
- Support for both hardware and simulation modes

## System Requirements
- Python 3.8+
- USB-connected RPLIDAR device (or run in mock mode)
- Arduino (optional - can run in mock mode)

## Installation

1. Clone the repository:
```
git clone https://github.com/sebaxe07/BovisaBabyLMmodule.git
cd BovisaBabyLMmodule
```

2. Install the required Python packages:
```
pip install -r requirements.txt
```

## Configuration
The system settings are defined in [**settings.yaml**](config/settings.yaml). Key configuration sections:

- **LIDAR:** Port, safety distance, update rates
- **Arduino:** Connection settings, mock mode settings
- **Robot:** Movement parameters and speeds
- **Camera:** Position tolerance and detection distances

## Running the System
To start the complete system:
```
python run.py
```

This will initialize:

- The main controller
- LIDAR processor
- Mock camera system
- Arduino interface (real or mock)
- Web-based visualization server
- To test only the LIDAR component:

## Web Interface
Once the system is running, access the visualization interface at:

```
http://localhost:5000
```

The interface provides:

- Real-time LIDAR scan visualization
- Obstacle tracking and detection
- Robot control buttons
- Human position simulation controls
- System status information

## Components
Sensors

- **LIDAR Processor** (sensors/lidar_processor.py): Processes LIDAR scan data and detects obstacles
- **Mock Camera** (sensors/camera_client.py): Simulates a camera tracking system

Control
- **Main Controller** (controllers/main_controller.py): Central control system that integrates all components
- **Arduino Interface** (motor/arduino_interface.py): Controls robot movement via Arduino (supports mock mode)

Visualization
- **Flask Interface** (utils/flask_visualization.py): Web-based visualization and control
- **LIDAR Visualizer** (utils/visualization.py): Matplotlib-based LIDAR data visualization

Utilities
- **Colored Logger** (utils/colored_logger.py): Provides color-coded logging

## Architecture
The system uses ZMQ for inter-process communication:

- Port 5555: LIDAR data publishing
- Port 5556: UI command channel
- Port 5557: Camera commands
- Port 5558: Camera tracking data

Components run as separate processes for stability and performance.


## Operating Modes

- **IDLE:** Robot is stationary
- **TRACKING:** Robot is tracking and following a target
- **SEARCH:** Robot is searching for a target
- **AVOIDING:** Robot is executing an obstacle avoidance maneuver

## Troubleshooting

- **LIDAR Connection Issues:** Check the port in [**settings.yaml**](config/settings.yaml) and ensure device is connected
- **Visualization Not Showing:** Make sure ZMQ processes are running correctly
- **Arduino Connection Failure:** The system will automatically fall back to mock mode

## License
This project is licensed under the MIT License - see the LICENSE file for details.

