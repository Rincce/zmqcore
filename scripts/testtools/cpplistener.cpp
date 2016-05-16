
#include <zmq.hpp>
#include "zhelpers.hpp"
#include <iostream>
#include <sstream>

int main (int argc, char *argv[])
{
    zmq::context_t context (1);

    std::cout << "Registering subscription with zmqcore.\n";
    zmq::socket_t requester(context, ZMQ_REQ);
    requester.connect("tcp://localhost:5555");
    // Send Multipart message
    s_sendmore(requester, "/chatter");
    s_sendmore(requester, "String");
    s_send(requester, "std_msgs.msg");
    std::string reply = s_recv(requester);
    std::cout << reply << "\n";
    //  Socket to talk to server
    std::cout << "Collecting updates from weather server…\n" << std::endl;
    zmq::socket_t subscriber (context, ZMQ_SUB);
    subscriber.connect("tcp://localhost:5556");

    subscriber.setsockopt(ZMQ_SUBSCRIBE, "/chatter", strlen ("/chatter"));

    while (true) {
        // Receive multipart message.
        std::string topic = s_recv(subscriber);
        std::string msg = s_recv(subscriber);

        std::cout << topic << " - " << msg << "\n";
    }
    return 0;
}
