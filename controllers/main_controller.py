# controllers/main_controller.py
import zmq
import time
from motor.arduino_interface import ArduinoInterface

class MainController:
    def __init__(self, config):
        self.config = config
        self.arduino = ArduinoInterface(config['arduino'])
        self._setup_communication()
        self.current_state = "IDLE"
        self.target_direction = None
        self.last_obstacles = []

    def _setup_communication(self):
        context = zmq.Context()
        # Subscribe to LIDAR data
        self.subscriber = context.socket(zmq.SUB)
        self.subscriber.connect("tcp://localhost:5555")
        self.subscriber.setsockopt_string(zmq.SUBSCRIBE, '')
        
        # Add camera subscription if needed
        # self.camera_sub = ...

    def _avoidance_strategy(self, obstacles):
        """Simple obstacle avoidance logic"""
        front_obstacles = [o for o in obstacles if abs(o['y']) < 0.2]
        if not front_obstacles:
            return "forward"
        
        # Find safest direction
        right_clear = all(o['x'] < -0.3 for o in front_obstacles)
        left_clear = all(o['x'] > 0.3 for o in front_obstacles)
        
        if right_clear:
            return "right"
        elif left_clear:
            return "left"
        else:
            return "stop"

    def process_messages(self):
        while True:
            try:
                msg = self.subscriber.recv_json(zmq.NOBLOCK)
                if msg['type'] == 'obstacles':
                    self.last_obstacles = msg['data']
                    
                    if self.current_state == "TRACKING":
                        action = self._avoidance_strategy(msg['data'])
                        self.arduino.send_command(action)
                        
            except zmq.Again:
                pass
            
            # Add human detection handling here
            # ...
            
            time.sleep(0.01)