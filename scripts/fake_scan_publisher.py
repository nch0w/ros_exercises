#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
import random
import numpy as np

def talker():
    pub = rospy.Publisher('fake_scan', LaserScan, queue_size=10)
    rospy.init_node('fake_scan_publisher', anonymous=True)
    rate = rospy.Rate(20) # 20hz
    while not rospy.is_shutdown():
        scan = LaserScan()
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = "base_link"
        scan.angle_min = rospy.get_param('angle_min', -2*np.pi/3)
        scan.angle_max = rospy.get_param('angle_max', 2*np.pi/3)
        scan.angle_increment = rospy.get_param('angle_increment', np.pi/300)
        scan.scan_time = rospy.get_param('scan_time', 0.05)
        scan.range_min = rospy.get_param('range_min', 1.0)
        scan.range_max = rospy.get_param('range_max', 10.0)
        scan.ranges = np.random.uniform(scan.range_min, scan.range_max, int((scan.angle_max - scan.angle_min)/scan.angle_increment) + 1)

        rospy.loginfo(scan)
        pub.publish(scan)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass