"""Test ZMQ-talker."""
import zmq
import time

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.connect('tcp://localhost:5557')

while True:
    socket.send_multipart(['/chatter',
                           "hello world %f" % time.time(),
                           'String',
                           'std_msgs.msg'])
    time.sleep(0.1)
