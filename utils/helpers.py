# Monitor ZMQ messages
import zmq
context = zmq.Context()
sock = context.socket(zmq.SUB)
sock.connect("tcp://localhost:5555")
sock.setsockopt_string(zmq.SUBSCRIBE, '')
while True:
    print(sock.recv_json())