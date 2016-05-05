#!/usr/bin/env python
"""The talker publishes chatter."""
import rospy
from std_msgs.msg import String


def talker():
    """Publish chatter at 10 hertz."""
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker')
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
