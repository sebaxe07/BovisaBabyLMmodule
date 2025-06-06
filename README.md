# Bovisa Baby Controller System

A modular robot control system with LIDAR-based obstacle detection, human tracking, autonomous charging, and motor control capabilities. The system provides a web-based visualization interface for real-time monitoring.

## Features

- LIDAR-based obstacle detection and tracking
- Camera-based human tracking with pose detection
- Robot movement control via Arduino interface
- April Tag detection for autonomous charging
- GPS geofencing capabilities
- Real-time visualization through web interface
- ZMQ-based inter-process communication
- Support for both hardware and simulation modes

## System Overview

The system runs across two Raspberry Pi devices:

- **Raspberry Pi 4**: Runs the main controller, LIDAR processor, and visualization
- **Raspberry Pi 5**: Handles computer vision (camera processing, human detection, April Tag detection)

## System Requirements

- Python 3.8+
- Two Raspberry Pi devices (Pi 4 and Pi 5)
- USB-connected RPLIDAR device (or run in mock mode)
- Arduino boards (optional - can run in mock mode)
- Camera (USB camera)

## Installation

1. Clone the repository on both Raspberry Pi devices:

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

- **LIDAR:** Port, safety distance, update rates, avoidance parameters
- **Arduino:** Connection settings, mock mode settings
- **Robot:** Movement parameters and speeds
- **Camera:** Position tolerance, detection distances, communication settings
- **GPS:** Mock settings, geofence configuration

## Running the System

The system is split across two devices and requires two different startup procedures:

### On Raspberry Pi 4 (Main Controller)

```
python run.py
```

This will initialize:

- The main controller
- LIDAR processor
- GPS processor
- Arduino interfaces
- Web-based visualization server

### On Raspberry Pi 5 (Camera Processing)

```
python run_camera.py
```

This will initialize:

- Camera client
- Human detection and tracking
- April Tag detection

## Web Interface

Once the system is running, access the visualization interface at:

```
http://raspberry-pi-4-ip:5000
```

The interface provides:

- Real-time LIDAR scan visualization
- Obstacle tracking and detection
- Robot control buttons
- Human position simulation controls
- System status information
- GPS location and geofence visualization

## Components

### Sensors

- **LIDAR Processor** (sensors/lidar_processor.py): Processes LIDAR scan data and detects obstacles
- **Camera Client** (sensors/camera_client.py): Handles human tracking and April Tag detection for charging
- **GPS Processor** (sensors/gps_processor.py): Processes GPS data and manages geofencing

### Control

- **Main Controller** (controllers/main_controller.py): Central control system that integrates all components
- **Arduino Interface** (motor/arduino_interface.py): Controls robot movement via Arduino (supports mock mode)
- **Multi Arduino Interface** (motor/multi_arduino_interface.py): Manages multiple Arduino connections

### Visualization

- **Flask Interface** (utils/flask_visualization/app.py): Web-based visualization and control
- **LIDAR Visualizer** (utils/visualization.py): Matplotlib-based LIDAR data visualization

### Utilities

- **Colored Logger** (utils/colored_logger.py): Provides color-coded logging

## Architecture

The system uses ZMQ for inter-process communication between components and between the two Raspberry Pi devices:

- Port 5555: LIDAR data publishing
- Port 5556: UI command channel
- Port 5557: Camera commands
- Port 5558: Camera tracking data
- Port 5559: Camera video stream
- Port 5560: GPS data publishing
- Port 5561: Communication module port

Components run as separate processes for stability and performance.

## Operating Modes

- **IDLE:** Robot is stationary
- **TRACKING:** Robot is tracking and following a human target
- **SEARCH:** Robot is searching for a human target
- **AVOIDING:** Robot is executing an obstacle avoidance maneuver
- **CHARGING:** Robot is detecting April Tags for autonomous charging
- **RETURNING:** Robot is returning to the geofence boundary when outside its allowed area

## Troubleshooting

- **LIDAR Connection Issues:** Check the port in [**settings.yaml**](config/settings.yaml) and ensure device is connected
- **Camera Not Working:** Check if run_camera.py is running on the Pi 5 and network settings are correct
- **Visualization Not Showing:** Make sure ZMQ processes are running correctly on both devices
- **Arduino Connection Failure:** The system will automatically fall back to mock mode
- **GPS Issues:** Check the geofence configuration and connection to GPS module

## Arduino Submodule

This project includes the [robotics-design](https://github.com/Dapringer/robotics-design) repository as a Git submodule in the `arduino` directory. This submodule handles the Arduino-specific code for localization functionality.

### Submodule Setup

When cloning this repository for the first time, you'll need to initialize the submodule:

```
git clone https://github.com/sebaxe07/BovisaBabyLMmodule.git
cd BovisaBabyLMmodule
git submodule init
git submodule update
```

Alternatively, clone with the `--recurse-submodules` flag:

```
git clone --recurse-submodules https://github.com/sebaxe07/BovisaBabyLMmodule.git
```

### Updating the Submodule

To update the submodule to the latest version:

```
git submodule update --remote arduino
```

**Note:** Access to the private robotics-design repository requires appropriate GitHub permissions.

## License

This project is licensed under the MIT License - see the LICENSE file for details.
