"""Test ZMQ-subscriber."""
import zmq
import cPickle

context = zmq.Context()

# Registering subscription with zmqcore.
socket = context.socket(zmq.REQ)
socket.connect('tcp://localhost:5555')

print "Registering subscription with zmqcore."
# Send multipart message: <topic>, <data_class>, <module>
socket.send_multipart([b'/chatter', b'String', b'std_msgs.msg'])
msg = socket.recv()
if int(msg) == -1:
    raise Exception("Subscription 'chatter' could not be registered.")

# Socket to talk to server
print "Connecting to chatter subscription."
sub_socket = context.socket(zmq.SUB)
sub_socket.connect('tcp://localhost:5556')
sub_socket.setsockopt_string(zmq.SUBSCRIBE, u'/chatter')

while True:
    msg = sub_socket.recv_multipart()
    print cPickle.loads(msg[1])
