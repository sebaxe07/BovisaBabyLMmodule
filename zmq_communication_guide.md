# Communication Guide: ZeroMQ & I2C

This guide outlines the communication structure for our distributed Raspberry Pi system, covering both ZeroMQ (network communication) and I2C (local device communication).

## ZeroMQ Communication

### Communication Framework

ZeroMQ (ZMQ) provides a messaging library for implementing publisher/subscriber patterns across network-connected devices.

### Network Setup

The system consists of three Raspberry Pi modules on the same network:

- **HumanDetection**: (`192.168.1.50`) - Computer vision and tracking functionality
- **LocalizationModule**: (`192.168.1.40`) - Localization and control functions
- **HandDetection**: (`192.168.1.30`) - Hand gesture detection and processing

### Communication Pattern

ZeroMQ implements a publisher/subscriber pattern:

- One module **publishes** messages on a specific address/port
- Other modules **subscribe** to receive those messages

### Existing Communication Channels

Currently implemented communication channels:

| From               | To                 | Address                 | Data            |
| ------------------ | ------------------ | ----------------------- | --------------- |
| HumanDetection     | LocalizationModule | tcp://192.168.1.50:5558 | Tracking data   |
| HumanDetection     | Any subscriber     | tcp://192.168.1.50:5559 | Video stream    |
| LocalizationModule | HumanDetection     | tcp://192.168.1.40:5557 | Camera commands |

### Implementation Guidelines

#### 1. ZeroMQ Installation:

```bash
sudo apt update
sudo apt install python3-pip
pip3 install pyzmq
```

#### 2. Setting up a Subscriber (Real example from LocalizationModule):

```python
# Initialize ZeroMQ context
context = zmq.Context()

# Add subscription for camera tracking data
camera_subscriber = context.socket(zmq.SUB)
camera_subscriber.connect("tcp://192.168.1.50:5558")
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

#### 3. Setting up a Publisher (Real example from HumanDetection):

```python
# Initialize ZeroMQ context
context = zmq.Context()

# Create publisher socket
publisher = context.socket(zmq.PUB)
publisher.bind("tcp://192.168.1.50:5558")

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

#### 4. Command Messages (Real example from LocalizationModule to HumanDetection):

```python
# Setup command publisher
camera_command_publisher = context.socket(zmq.PUB)
camera_command_publisher.bind("tcp://192.168.1.40:5557")

# Start searching for humans
camera_command_publisher.send_json({
    'command': 'SEARCH'
})

# Stop searching
camera_command_publisher.send_json({
    'command': 'STOP'
})
```

#### 5. Video Stream Handling (From HumanDetection):

```python
# Setup video publisher
video_publisher = context.socket(zmq.PUB)
video_publisher.bind("tcp://192.168.1.50:5559")

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
video_subscriber.connect("tcp://192.168.1.50:5559")
video_subscriber.setsockopt_string(zmq.SUBSCRIBE, '')

topic, frame_data, timestamp = video_subscriber.recv_multipart()
frame = cv2.imdecode(
    np.frombuffer(frame_data, dtype=np.uint8),
    cv2.IMREAD_COLOR
)
# Process the frame as needed
```

## I2C Communication

### Communication Framework

I2C (Inter-Integrated Circuit) is a serial communication protocol used for connecting microcontrollers and peripherals. In our project, I2C connects the Raspberry Pi (master) with Arduino (slave).

### Hardware Setup

- **Raspberry Pi**: Acts as the I2C master
- **Arduino**: Acts as the I2C slave (address: `0x08`)
- **Connections**:
  - Pin 3 (SDA) on Raspberry Pi to Arduino Pin 20 (SDA)
  - Pin 5 (SCL) on Raspberry Pi to Arduino Pin 21 (SCL)
  - Common ground between devices

### Communication Pattern

I2C uses a master-slave architecture:

- The Raspberry Pi (master) initiates all communications
- The Arduino (slave) responds to commands from the master

### Command Structure

Our system currently uses string-based commands sent from Raspberry Pi to Arduino:

| Command Type | Format            | Description                                    |
| ------------ | ----------------- | ---------------------------------------------- |
| Proportional | `-10.0` to `10.0` | Controls differential steering speed/direction |
| Stop         | `100.0`           | Stops the motors and sets stopOverwrite flag   |
| Found        | `101.0`           | Stops and marks human as detected in front     |
| Left         | `102.0`           | Turns the robot left                           |
| Right        | `103.0`           | Turns the robot right                          |
| Forward      | `104.0`           | Moves the robot forward                        |
| Backward     | `105.0`           | Moves the robot backward                       |

### Implementation Guidelines

#### 1. I2C Setup on Raspberry Pi:

```bash
# Enable I2C interface on Raspberry Pi
sudo raspi-config
# Navigate to: Interface Options > I2C > Yes

# Install required packages
sudo apt update
sudo apt install python3-smbus i2c-tools

# Check I2C devices
sudo i2cdetect -y 1
```

#### 2. Sending Commands from Raspberry Pi (from ArduinoInterface):

```python
from smbus2 import SMBus

# Send string command to Arduino
def send_string(text, address=0x08):
    # Convert string to bytes
    string_bytes = bytearray(text.encode('utf-8'))
    # Limit to I2C buffer size
    if len(string_bytes) > 32:
        string_bytes = string_bytes[:32]

    with SMBus(1) as bus:
        try:
            # Send string as a block of bytes
            bus.write_i2c_block_data(address, 0, list(string_bytes))
            print(f"Sent string: '{text}' to Arduino")
        except Exception as e:
            print(f"Error sending string data: {e}")

# Sending directional commands
def send_command(command):
    # Command can be a float or string
    if isinstance(command, float):
        # Send float value directly (-10.0 to 10.0)
        if -10.0 <= command <= 10.0:
            command_str = f"{command:.2f}"
            send_string(command_str)
    elif isinstance(command, str):
        # Map string commands to float values
        command_map = {
            "stop": 100.0,
            "found": 101.0,
            "left": 102.0,
            "right": 103.0,
            "forward": 104.0,
            "backward": 105.0,
        }
        command_value = command_map.get(command.lower(), 100.0)  # Default to stop
        send_string(f"{command_value:.2f}")
```

#### 3. Receiving on Arduino Side (from UltraSonicAndLED.ino):

```cpp
#include <Wire.h>

#define I2C_ADDRESS 0x08

char dataReceived[32];                // Buffer to store received data
volatile bool newDataAvailable = false;

void setup() {
  Serial.begin(9600);
  Wire.begin(I2C_ADDRESS);      // Join I2C bus as slave
  Wire.onReceive(receiveEvent); // Register receive handler
  Serial.println("Arduino I2C Slave ready...");
}

void receiveEvent(int howMany) {
  int i = 0;
  while (Wire.available() > 0 && i < sizeof(dataReceived) - 1) {
    char c = Wire.read();
    if (c != '\0') { // skip null characters
      dataReceived[i++] = c;
    }
  }
  dataReceived[i] = '\0';  // Null-terminate the string
  newDataAvailable = true;
}

void processI2CData() {
  if (newDataAvailable) {
    newDataAvailable = false; // Reset the flag immediately

    float receivedValue = atof(dataReceived); // Convert string to float
    int intValue = (int)receivedValue;

    // Handle proportional speed control (-10 to 10)
    if (receivedValue > -10 && receivedValue < 10 && receivedValue != 0) {
      driveWithProportionalSpeed(receivedValue);
      return;
    }

    // Handle discrete commands (100-105)
    switch (intValue) {
      case 100: // Stop both motors
        driveRobot(0);
        humanInFront = false;
        stopOverwrite = true;
        break;
      case 101: // Human found
        humanInFront = true;
        driveRobot(0);
        stopOverwrite = false;
        break;
      case 102: // Turn left
        driveRobot(3);
        humanInFront = false;
        stopOverwrite = false;
        break;
      case 103: // Turn right
        driveRobot(4);
        humanInFront = false;
        stopOverwrite = false;
        break;
      case 104: // Move forward
        driveRobot(1);
        humanInFront = false;
        stopOverwrite = false;
        break;
      case 105: // Move backward
        driveRobot(2);
        humanInFront = false;
        stopOverwrite = false;
        break;
    }
  }
}

void loop() {
  // Process I2C data if available
  processI2CData();

  // Rest of the loop
  // ...
}
```

#### 4. Implementing Two-Way Communication:

> **Note:** This section describes future planned functionality that has not been implemented or tested yet. The actual implementation may differ based on testing and specific requirements.

##### Planned Arduino to Raspberry Pi communication:

```cpp
// On Arduino: Send data back to Raspberry Pi when requested
void requestEvent() {
  // This function is called when the master (Raspberry Pi) requests data

  // Example: Prepare sensor data to send
  char buffer[32];

  // Format data as a string
  // You might want to include sensor readings, motor status, or error states
  sprintf(buffer, "DIST:%d,OBST:%d,BAT:%d",
          (int)(sensorFront.getDistance() * 10),  // Distance in mm (multiplied by 10 to avoid floats)
          (wallFront ? 1 : 0),                    // Obstacle detected flag
          analogRead(A0) / 10);                   // Battery level reading divided by 10

  // Send the buffer over I2C
  Wire.write(buffer);
}

void setup() {
  // ...existing code...

  // Register the request handler for responding to master
  Wire.onRequest(requestEvent);

  // ...existing code...
}
```

##### Planned Raspberry Pi implementation to request and process data:

```python
def get_arduino_data(self, address=0x08):
    """
    Request and read data from the Arduino.
    Returns a dictionary of sensor values or None if an error occurred.

    Note: This is a planned implementation and has not been tested.
    """
    try:
        with SMBus(1) as bus:
            # Request and read up to 32 bytes from the Arduino
            # The Arduino's requestEvent function will be triggered
            raw_data = bus.read_i2c_block_data(address, 0, 32)

            # Convert bytes to string (stop at null terminator)
            data_str = ""
            for b in raw_data:
                if b == 0:
                    break
                data_str += chr(b)

            # Parse the key-value format
            # Expected format example: "DIST:250,OBST:1,BAT:42"
            parsed_data = {}
            if data_str:
                pairs = data_str.split(',')
                for pair in pairs:
                    if ':' in pair:
                        key, value = pair.split(':')
                        # Convert values to appropriate types
                        if '.' in value:
                            # It's probably a float
                            parsed_data[key] = float(value)
                        else:
                            # It's probably an integer
                            parsed_data[key] = int(value)

            log_info("ARDUINO", f"Received data from Arduino: {parsed_data}")
            return parsed_data

    except Exception as e:
        log_error("ARDUINO", f"Error reading data from Arduino: {e}")
        return None
```

##### Implementation considerations for two-way communication:

1. **Sequence timing:** When implementing two-way communication, be careful about the timing between write and read operations. Allow sufficient time for the Arduino to process requests.

2. **Error handling:** Both sides need robust error handling as I2C communication can fail if the bus is busy or connections are unstable.

3. **Buffer limitations:** I2C typically has small buffer sizes (32 bytes on Arduino). Complex data may need to be split into multiple transmissions.

4. **Testing approach:** When implementing this functionality:

   - Start with simple ping-pong tests (request/response with minimal data)
   - Gradually increase complexity of data being transferred
   - Test under various conditions (different loads, distances between devices)

5. **Polling vs. Event-based:** The current design uses polling from the Raspberry Pi. Consider whether event-based notifications would be more efficient for your specific use case.

### I2C Best Practices

1. **Buffer Size Limits** - Arduino's buffer is limited to 32 bytes

2. **String-based Protocol** - Our implementation uses string commands converted to float values

3. **Error Handling** - Always use try/except when communicating over I2C

```python
try:
    # I2C communication code
    bus.write_i2c_block_data(address, 0, data)
except OSError as e:
    print(f"I2C communication error: {e}")
    # Handle disconnection, etc.
```

4. **Command Processing** - Process I2C data in the main loop rather than directly in the receive handler
