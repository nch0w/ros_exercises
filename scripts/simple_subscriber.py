#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import numpy as np
from functools import partial

def callback(data, pub=None):
    log_num = np.log(data.data)
    pub.publish(log_num)
    rospy.loginfo(log_num)

def listener():
    rospy.init_node('simple_subscriber', anonymous=True)
    pub = rospy.Publisher('random_float_log', Float32, queue_size=10)
    rospy.Subscriber('my_random_float', Float32, partial(callback, pub=pub))
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
