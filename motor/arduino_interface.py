import smbus
import time
import logging
from threading import Thread, Event
from utils.colored_logger import log_info, log_error, log_debug

class ArduinoInterface:
    def __init__(self, config):
        self.mock_mode = config.get('mock_mode', False)
        print(f"Mock mode: {self.mock_mode}")
        self.logger = logging.getLogger('arduino_interface')
        
        if self.mock_mode:
            self.logger.info("Running in MOCK mode - no actual hardware will be used")
            self._setup_mock()
        else:
            try:
                self.bus = smbus.SMBus(config['bus_number'])
                self.address = config['arduino_address']
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

    def _initialize_arduino(self):
        """Send an initialization signal to the Arduino."""
        try:
            self.bus.write_byte(self.address, 0)  # Send a reset/init signal
            time.sleep(0.1)  # Allow Arduino to process
        except Exception as e:
            self.logger.error(f"Error initializing Arduino: {e}")

    def send_command(self, command):
        """
        Send a movement command to the Arduino.
        Commands are represented as integers:
        0 = stop, 1 = forward, 2 = backward, 3 = left, 4 = right
        """
        if isinstance(command, str):
            command_map = {
                "stop": 0,
                "forward": 1,
                "backward": 2,
                "left": 3,
                "right": 4
            }
            command = command_map.get(command.lower(), 0)  # Default to stop if invalid
            
        if command not in range(5):  # Validate command range
            self.logger.warning(f"Invalid command value: {command}. Defaulting to stop.")
            command = 0
            
        if self.mock_mode:
            self.logger.debug(f"MOCK: Sending command {command}")
            self.mock_state['command'] = command
        else:
            try:
                self.bus.write_byte(self.address, command)
            except Exception as e:
                self.logger.error(f"Error sending command to Arduino: {e}")
                
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