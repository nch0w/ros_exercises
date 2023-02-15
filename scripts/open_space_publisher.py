#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
import numpy as np
from functools import partial

def callback(data, pub=None):
    pub_distance, pub_angle = pub
    max_range_index = np.argmax(data.ranges)
    max_range_angle = data.angle_min + max_range_index * data.angle_increment
    pub_distance.publish(data.ranges[max_range_index])
    pub_angle.publish(max_range_angle)
    
    rospy.loginfo("max range: %f, angle: %f", data.ranges[max_range_index], max_range_angle)

def listener():
    rospy.init_node('open_space_publisher', anonymous=True)
    pub_distance = rospy.Publisher('open_space/distance', Float32, queue_size=10)
    pub_angle = rospy.Publisher('open_space/angle', Float32, queue_size=10)
    rospy.Subscriber('fake_scan', LaserScan, partial(callback, pub=[pub_distance, pub_angle]))
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
