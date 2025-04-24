# ZeroMQ Communication Guide

This guide outlines the ZeroMQ communication structure for our distributed Raspberry Pi system.

## Communication Framework

We're using **ZeroMQ** (ZMQ) for inter-device communication. It provides a messaging library for implementing publisher/subscriber patterns across network-connected devices.

## Network Setup

The system consists of three Raspberry Pi modules on the same network:

- **HumanDetection**: (`192.168.10.1`) - Computer vision and tracking functionality
- **LocalizationModule**: (`192.168.10.2`) - Localization and control functions
- **HandDetection**: (`192.168.10.3`) - Hand gesture detection and processing

## Communication Pattern

ZeroMQ implements a publisher/subscriber pattern:

- One module **publishes** messages on a specific address/port
- Other modules **subscribe** to receive those messages

## Existing Communication Channels

Currently implemented communication channels:

| From               | To                 | Address                 | Data            |
| ------------------ | ------------------ | ----------------------- | --------------- |
| HumanDetection     | LocalizationModule | tcp://192.168.10.1:5558 | Tracking data   |
| HumanDetection     | Any subscriber     | tcp://192.168.10.1:5559 | Video stream    |
| LocalizationModule | HumanDetection     | tcp://192.168.10.2:5557 | Camera commands |

## Implementation Guidelines

### 1. ZeroMQ Installation:

```bash
sudo apt update
sudo apt install python3-pip
pip3 install pyzmq
```

### 2. Setting up a Subscriber (Real example from LocalizationModule):

```python
# Initialize ZeroMQ context
context = zmq.Context()

# Add subscription for camera tracking data
camera_subscriber = context.socket(zmq.SUB)
camera_subscriber.connect("tcp://192.168.10.1:5558")
camera_subscriber.setsockopt_string(zmq.SUBSCRIBE, '')

# Receive and process messages
try:
    camera_msg = camera_subscriber.recv_json(zmq.NOBLOCK)
    if camera_msg['type'] == 'TRACKING':
        # Process tracking data
        target_position = camera_msg['x_position']
        target_distance = camera_msg['distance']
        human_id = camera_msg['human_id']
        # Take action based on this data
    elif camera_msg['type'] == 'NOTFOUND':
        # Handle when target is not found
        # ...
except zmq.Again:
    # No message available
    pass
```

### 3. Setting up a Publisher (Real example from HumanDetection):

```python
# Initialize ZeroMQ context
context = zmq.Context()

# Create publisher socket
publisher = context.socket(zmq.PUB)
publisher.bind("tcp://192.168.10.1:5558")

# Example message when tracking a person
message = {
    "type": "TRACKING",
    "x_position": x_value,  # lateral position from -10 to 10
    "distance": distance,    # distance in meters
    "human_id": track_id     # unique identifier for the tracked person
}
publisher.send_json(message)

# Example message when person not found
message = {
    "type": "NOTFOUND",
    "x_position": 0,
    "distance": 0,
    "human_id": 0
}
publisher.send_json(message)
```

### 4. Command Messages (Real example from LocalizationModule to HumanDetection):

```python
# Setup command publisher
camera_command_publisher = context.socket(zmq.PUB)
camera_command_publisher.bind("tcp://192.168.10.2:5557")

# Start searching for humans
camera_command_publisher.send_json({
    'command': 'SEARCH'
})

# Stop searching
camera_command_publisher.send_json({
    'command': 'STOP'
})
```

### 5. Video Stream Handling (From HumanDetection):

```python
# Setup video publisher
video_publisher = context.socket(zmq.PUB)
video_publisher.bind("tcp://192.168.10.1:5559")

# Send video frame
def send_frame(frame):
    # Encode the frame as JPEG
    _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])

    # Send the frame with metadata
    video_publisher.send_multipart([
        b"frame",
        buffer.tobytes(),
        f"{int(time.time() * 1000)}".encode('utf-8')  # Timestamp
    ])

# On subscriber side to receive frames
video_subscriber = context.socket(zmq.SUB)
video_subscriber.connect("tcp://192.168.10.1:5559")
video_subscriber.setsockopt_string(zmq.SUBSCRIBE, '')

topic, frame_data, timestamp = video_subscriber.recv_multipart()
frame = cv2.imdecode(
    np.frombuffer(frame_data, dtype=np.uint8),
    cv2.IMREAD_COLOR
)
# Process the frame as needed
```

## Communication Best Practices

1. **Error Handling** - Implement proper error handling for network interruptions

2. **Non-blocking Receives** - Use `zmq.NOBLOCK` with try/except to prevent program blocking:

```python
try:
    message = socket.recv_json(zmq.NOBLOCK)
    # Process message
except zmq.Again:
    # No message available
    pass
```

3. **Threaded Communication** - Run publishers/subscribers in separate threads for responsiveness:

```python
from threading import Thread

# Example from camera_client.py
stream_thread = Thread(target=self._stream_video_loop)
stream_thread.daemon = True  # Thread will exit when main program exits
stream_thread.start()
```
