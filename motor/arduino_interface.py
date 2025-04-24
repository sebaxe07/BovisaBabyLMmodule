import smbus
import time
import logging
from threading import Thread, Event
import struct 
from smbus2 import SMBus
from utils.colored_logger import log_info, log_error, log_debug

class ArduinoInterface:
    def __init__(self, config):
        self.mock_mode = config['mock_mode']

        self.logger = logging.getLogger('arduino_interface')
        
        if self.mock_mode:
            self.logger.info("Running in MOCK mode - no actual hardware will be used")
            self._setup_mock()
        else:
            try:
                self.address = config['I2C_ADDRESS']
                self._initialize_arduino()
                self.logger.info("Connected to Arduino via I2C")
            except Exception as e:
                self.logger.warning(f"Failed to connect to Arduino: {e}. Falling back to mock mode")
                self.mock_mode = True
                self._setup_mock()

    def _setup_mock(self):
        """Set up mock environment for testing without hardware"""
        self.mock_state = {
            'command': 0,  # Current command (0=stop)
            'motor_left': 0,  # Left motor speed (-100 to 100)
            'motor_right': 0,  # Right motor speed (-100 to 100)
            'sensors': {
                'front_distance': 100.0,  # cm
                'battery': 12.0  # volts
            }
        }
        
        # Start a thread to simulate Arduino behavior
        self._stop_event = Event()
        self._mock_thread = Thread(target=self._mock_loop)
        self._mock_thread.daemon = True
        self._mock_thread.start()
        
    def _mock_loop(self):
        """Simulate Arduino behavior in a background thread"""
        while not self._stop_event.is_set():
            # Simulate motor movement based on current command
            if self.mock_state['command'] == 1:  # Forward
                self.mock_state['motor_left'] = 75
                self.mock_state['motor_right'] = 75
            elif self.mock_state['command'] == 2:  # Backward
                self.mock_state['motor_left'] = -75
                self.mock_state['motor_right'] = -75
            elif self.mock_state['command'] == 3:  # Left
                self.mock_state['motor_left'] = -50
                self.mock_state['motor_right'] = 50
            elif self.mock_state['command'] == 4:  # Right
                self.mock_state['motor_left'] = 50
                self.mock_state['motor_right'] = -50
            else:  # Stop or invalid
                self.mock_state['motor_left'] = 0
                self.mock_state['motor_right'] = 0
            
            # Simulate battery drain (very slow)
            self.mock_state['sensors']['battery'] -= 0.0001
            
            # Sleep to avoid hogging CPU
            time.sleep(0.1)

    def send_string(self, text):
        """Send a string value to the Arduino via I2C."""
        # Convert string to bytes
        string_bytes = bytearray(text.encode('utf-8'))
        # Limit to a reasonable length if needed (Arduino buffer size considerations)
        if len(string_bytes) > 32:  # 32 bytes is a common I2C buffer size
            string_bytes = string_bytes[:32]
        
        with SMBus(1) as bus:
            try:
                # Send string as a block of bytes
                log_debug("ARDUINO", f"Sending string bytes: {list(string_bytes)}")
                bus.write_i2c_block_data(self.address, 0, list(string_bytes))
                log_info("ARDUINO", f"Sent string: '{text}' to Arduino at address 0x{self.address:02X}")
            except Exception as e:
                log_error("ARDUINO", f"Error sending string data: {e}")

    def _initialize_arduino(self):
        """Send an initialization signal to the Arduino."""
        try:
            self.send_string("100.0")  # Send stop command to Arduino
            time.sleep(0.1)  # Allow Arduino to process
        except Exception as e:
            self.logger.error(f"Error initializing Arduino: {e}")

    def send_command(self, command):
        """
        Send a movement command to the Arduino.
        Commands can be:
        - Float values from -10.0 to 10.0 (will be converted to strings)
        - String values: "stop", "found", "left", "right", "forward", "backward"
        """
        log_info("ARDUINO", f"Receiving command: {command}")

        # Handle float commands by converting to string
        if isinstance(command, float):
            if -10.0 <= command <= 10.0:
                # Convert float to string with specified precision
                command_str = f"{command:.2f}"
                if self.mock_mode:
                    log_info("ARDUINO", f"Mock mode - setting float command to {command}")
                    self.mock_state['command'] = command
                else:
                    try:
                        log_info("ARDUINO", f"Sending float as string command: {command_str}")
                        self.send_string(command_str)
                    except Exception as e:
                        log_error("ARDUINO", f"Error sending float command: {e}")
            else:
                log_error("ARDUINO", f"Invalid float value: {command}. Defaulting to stop.")
                command = "stop"
        
        # Handle string commands
        
        # Handle string commands
        elif isinstance(command, str):
            command_map = {
                "stop": 100.0,
                "found": 101.0,
                "left": 102.0,
                "right": 103.0,
                "forward": 104.0,
                "backward": 105.0,
            }
            command = command_map.get(command.lower(), 100.0)  # Default to stop if invalid
            command_str = f"{command:.2f}"

            if self.mock_mode:
                log_info("ARDUINO", f"Mock mode - setting command to {command}")
                self.mock_state['command'] = command
            else:
                try:
                    log_info("ARDUINO", f"Sending command {command} to Arduino")
                    self.send_string(command_str)
                except Exception as e:
                    log_error("ARDUINO", f"Error sending command: {e}")
            
        else:
            log_error("ARDUINO", f"Unsupported command type: {type(command)}. Ignoring.")

           
    def get_sensor_data(self):
        """
        Get sensor readings from Arduino.
        In mock mode, returns simulated values.
        In hardware mode, would query the Arduino for data.
        """
        if self.mock_mode:
            return self.mock_state['sensors']
        else:
            # In a real implementation, you would read from the Arduino
            # Example: return self._read_sensor_data_from_arduino()
            # For now, returning None to indicate not implemented
            return None
            
    def cleanup(self):
        """Clean up resources"""
        if self.mock_mode:
            self._stop_event.set()
            if self._mock_thread.is_alive():
                self._mock_thread.join(timeout=1.0)
        else:
            # Send stop command before closing
            try:
                self.send_command("stop")
            except:
                pass