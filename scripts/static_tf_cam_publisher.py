#!/usr/bin/env python

import rospy
import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('static_tf_cam_publisher')
    
    tf_buffer = tf2_ros.Buffer()
    broadcaster = tf2_ros.StaticTransformBroadcaster()

    left_cam_pose = geometry_msgs.msg.TransformStamped()
    left_cam_pose.header.stamp = rospy.Time.now()
    left_cam_pose.header.frame_id = 'base_link_gt'
    left_cam_pose.child_frame_id = 'left_camera'
    left_cam_pose.transform.translation.x -= 0.05
    left_cam_pose.transform.rotation = geometry_msgs.msg.Quaternion(0, 0, 0, 1)
    # broadcaster.sendTransform(left_cam_pose)

    right_cam_pose = geometry_msgs.msg.TransformStamped()
    right_cam_pose.header.stamp = rospy.Time.now()
    right_cam_pose.header.frame_id = 'base_link_gt'
    right_cam_pose.child_frame_id = 'right_camera'
    right_cam_pose.transform.translation.x += 0.05
    right_cam_pose.transform.rotation = geometry_msgs.msg.Quaternion(0, 0, 0, 1)
    broadcaster.sendTransform((left_cam_pose, right_cam_pose))

    rospy.loginfo('Published static transforms')
    rospy.spin()
