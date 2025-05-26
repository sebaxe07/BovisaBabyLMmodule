import time
import logging
import serial
from threading import Thread, Event
from utils.colored_logger import log_info, log_error, log_debug

class ArduinoReader:
    """
    Generic class for reading data from Arduino devices over serial
    """
    def __init__(self, config, arduino_type):
        """
        Initialize an Arduino reader
        
        Args:
            config (dict): Configuration for this Arduino
            arduino_type (str): Type of Arduino (for logging purposes)
        """
        self.mock_mode = config['mock_mode']
        self.arduino_type = arduino_type
        
        if self.mock_mode:
            log_info(f"ARDUINO_{self.arduino_type.upper()}", f"Running {arduino_type} Arduino in MOCK mode - no actual hardware will be used")
        else:
            try:
                self.port = config['port']
                self.baudrate = config['baudrate']
                
                self.serial = serial.Serial(
                    port=self.port,
                    baudrate=self.baudrate,
                    timeout=1
                )
                time.sleep(2)  # Wait for Arduino to reset after serial connection
                log_info(f"ARDUINO_{self.arduino_type.upper()}",f"Connected to {arduino_type} Arduino via Serial on port {self.port}")
                
                # Start a thread to read from the serial port
                self._stop_event = Event()
                self._serial_read_thread = Thread(target=self._read_serial_data)
                self._serial_read_thread.daemon = True
                self._serial_read_thread.start()
                
            except Exception as e:
                log_error(f"ARDUINO_{self.arduino_type.upper()}",f"Failed to connect to {arduino_type} Arduino: {e}. Falling back to mock mode")
                self.mock_mode = True
    
    def _read_serial_data(self):
        """Read data from Arduino in a background thread."""
        while not self._stop_event.is_set():
            if self.mock_mode:
                time.sleep(0.1)
                continue
                
            try:
                if self.serial.in_waiting > 0:
                    data = self.serial.readline().decode('utf-8').strip()
                    if data:
                        # Check if the data Starts with "Emergency" and print log_error
                        if data.startswith("Emergency"):
                            log_error(f"ARDUINO_{self.arduino_type.upper()}", f"Received emergency signal: {data}")
                        else:
                            # Log all data received from the Arduino
                            log_debug(f"ARDUINO_{self.arduino_type.upper()}", f"Received: {data}")
                    
            except Exception as e:
                log_error(f"ARDUINO_{self.arduino_type.upper()}", f"Error reading from Arduino: {e}")
                time.sleep(0.1)  # Prevent tight error loop

    def cleanup(self):
        """Clean up resources"""
        if hasattr(self, '_stop_event'):
            self._stop_event.set()
        
        if not self.mock_mode and hasattr(self, 'serial') and self.serial.is_open:
            try:
                self.serial.close()
                log_info(f"ARDUINO_{self.arduino_type.upper()}", f"Closed serial connection on port {self.port}")
            except Exception as e:
                log_error(f"ARDUINO_{self.arduino_type.upper()}", f"Error during cleanup: {e}")
            
            # Wait for serial read thread to finish
            if hasattr(self, '_serial_read_thread') and self._serial_read_thread.is_alive():
                self._serial_read_thread.join(timeout=1.0)


class SensorArduino(ArduinoReader):
    """
    Class for interfacing with the Sensor Arduino
    """
    def __init__(self, config):
        super().__init__(config, "SENSOR")
        

class PowerArduino(ArduinoReader):
    """
    Class for interfacing with the Power Supply Arduino
    """
    def __init__(self, config):
        super().__init__(config, "POWER")

# Manager class to handle all Arduino interfaces
class ArduinoManager:
    """
    Class for managing all Arduino interfaces
    """
    def __init__(self, config):
        """
        Initialize the Arduino Manager with configuration
        
        Args:
            config (dict): Configuration containing settings for all Arduinos
        """
        self.motor_arduino = None
        self.sensor_arduino = None
        self.power_arduino = None
        
        # Initialize motor Arduino if configured
        if 'arduino' in config:
            from motor.arduino_interface import ArduinoInterface
            self.motor_arduino = ArduinoInterface(config['arduino'])
            log_info("ARDUINO_MANAGER", "Motor Arduino interface initialized")
        
        # Initialize sensor Arduino if configured
        if 'arduino_sensor' in config:
            self.sensor_arduino = SensorArduino(config['arduino_sensor'])
            log_info("ARDUINO_MANAGER", "Sensor Arduino interface initialized")
        
        # # Initialize power Arduino if configured
        if 'arduino_power' in config:
            self.power_arduino = PowerArduino(config['arduino_power'])
            log_info("ARDUINO_MANAGER", "Power Arduino interface initialized")
    
    def cleanup(self):
        """Clean up all Arduino interfaces"""
        if self.motor_arduino:
            self.motor_arduino.cleanup()
        
        if self.sensor_arduino:
            self.sensor_arduino.cleanup()
        
        if self.power_arduino:
            self.power_arduino.cleanup()
        
        log_info("ARDUINO_MANAGER", "All Arduino interfaces cleaned up")
