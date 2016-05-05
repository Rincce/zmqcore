#!/usr/bin/env python
"""
ROS-node to allow communication between ROS and ZMQ.

Created on May 5 6, 2016

@author: U{Rincce<rince.wind@gmx.at>}
"""

import zmq
import cPickle
import rospy


def load_class(module, classname):
    """
    Load class from module.

    @param module: I{(str)} Module in which the class can me found.
    @param classname: I{(str)} Name of the class that should be loaded.
    """
    mods = module.split('.')
    mod = __import__(mods[0])
    for m in mods[1:]:
        mod = getattr(mod, m)
    return getattr(mod, classname)


class ZMQCore(object):
    """zmqcore class."""

    def __init__(self):
        """Initialize ZMQCore class."""
        self.subs = {}  # dict of registered subscribers.
        self.pubs = {}  # dict of registered publishers.
        context = zmq.Context()
        # Create ZMQ registering service.
        self.reg_service = context.socket(zmq.REP)
        self.reg_service.bind('tcp://*:5555')
        # Create ZMQ publisher socket.
        self.pub_socket = context.socket(zmq.PUB)
        self.pub_socket.bind('tcp://*:5556')
        # Create ZMQ subscriber socket.
        self.sub_socket = context.socket(zmq.SUB)
        self.sub_socket.bind('tcp://*:5557')
        self.sub_socket.setsockopt(zmq.SUBSCRIBE, '')
        # Initiallize poller to deal with multiple sockets
        self.poller = zmq.Poller()
        self.poller.register(self.reg_service, zmq.POLLIN)
        self.poller.register(self.sub_socket, zmq.POLLIN)

    def poll(self):
        """Poll for registered sockets."""
        socks = dict(self.poller.poll())
        for sock in socks:
            if sock == self.reg_service:
                msg = self.reg_service.recv_multipart()
                # Check if the topic is acturally published.
                topic_list = rospy.get_published_topics()
                topics = [x[0] for x in topic_list]
                if msg[0] not in topics:
                    self.reg_service.send(b'-1')
                else:
                    self.reg_service.send(b'0')
                    data_class = load_class(msg[2], msg[1])
                    sub = rospy.Subscriber(msg[0],
                                           data_class,
                                           self.handle_subscription,
                                           msg[0])
                    self.subs[msg[0]] = sub
            if sock == self.sub_socket:
                msg = self.sub_socket.recv_multipart()
                if msg[0] not in self.pubs:
                    data_class = load_class(msg[3], msg[2])
                    pub = rospy.Publisher(msg[0], data_class, queue_size=10)
                    self.pubs[msg[0]] = pub
                self.pubs[msg[0]].publish(msg[1])

    def handle_subscription(self, data, sub):
        """
        Relay subscription data through ZMQ.

        @param data: The data that is published under the subscribed topic.
        @param sub: I{(str)} The subscription name.
        """
        self.pub_socket.send_multipart(
                            [sub,
                             cPickle.dumps(data,
                                           cPickle.HIGHEST_PROTOCOL)]
                                      )


def zmqcore_run():
    """
    Initialize ROS-node 'zmqcore'.

    At first this node relays the messages published in ROS to
    ZMQ.  As the program grows, messages published from ZMQ will
    be sent to ROS too.
    """
    rospy.init_node('zmqcore')
    zmqcore = ZMQCore()
    rate = rospy.Rate(60)  # 60 hz
    while not rospy.is_shutdown():
        zmqcore.poll()
        rate.sleep()

if __name__ == '__main__':
    zmqcore_run()
