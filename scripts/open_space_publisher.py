#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from ros_exercises.msg import OpenSpace
import numpy as np
from functools import partial

def callback(data, pub=None):
    max_range_index = np.argmax(data.ranges)
    max_range_angle = data.angle_min + max_range_index * data.angle_increment
    
    open_space = OpenSpace()
    open_space.angle = max_range_angle
    open_space.distance = data.ranges[max_range_index]
    pub.publish(open_space)
    
    rospy.loginfo("max range: %f, angle: %f", data.ranges[max_range_index], max_range_angle)

def listener():
    rospy.init_node('open_space_publisher', anonymous=True)
    pub = rospy.Publisher('open_space', OpenSpace, queue_size=10)
    rospy.Subscriber('fake_scan', LaserScan, partial(callback, pub=pub))
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
