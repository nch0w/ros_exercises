#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
import numpy as np
import tf2_ros
import tf.transformations as tft
import geometry_msgs.msg

# left_cam_translation = tf.transformations.translation_matrix([-0.05, 0, 0])
# right_cam_translation = tf.transformations.translation_matrix([0.10, 0, 0])
left_cam_translation = tft.translation_matrix([0.05, 0, 0])

if __name__ == '__main__':
    rospy.init_node('dynamic_tf_cam_publisher')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    broadcaster = tf2_ros.TransformBroadcaster()

    r = rospy.Rate(10)

    while not rospy.is_shutdown():
        try: 
            transform = tfBuffer.lookup_transform('world', 'left_cam', rospy.Time(0))
            translation = np.array([transform.transform.translation.x,
                                          transform.transform.translation.y,
                                          transform.transform.translation.z])
            rotation = np.array([transform.transform.rotation.x,
                                       transform.transform.rotation.y,
                                       transform.transform.rotation.z,
                                       transform.transform.rotation.w])
            transform_matrix = tft.concatenate_matrices(tft.translation_matrix(translation),
                                                               tft.quaternion_matrix(rotation))

            
            left_cam_matrix = np.dot(transform_matrix, np.linalg.inv(left_cam_translation))
            left_cam_transform = geometry_msgs.msg.TransformStamped()
            left_cam_transform.header.frame_id = 'world'
            left_cam_transform.child_frame_id = 'base_link_gt_2'
            left_cam_transform.header.stamp = rospy.Time.now()
            left_cam_transform.transform.translation.x = left_cam_matrix[0,3]
            left_cam_transform.transform.translation.y = left_cam_matrix[1,3]
            left_cam_transform.transform.translation.z = left_cam_matrix[2,3]
            left_cam_transform.transform.rotation = geometry_msgs.msg.Quaternion(*tft.quaternion_from_matrix(left_cam_matrix))

            broadcaster.sendTransform(left_cam_transform)
            r.sleep()
            rospy.loginfo(left_cam_transform)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(e)
            continue