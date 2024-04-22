#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, TransformStamped, Quaternion, Vector3
import tf2_ros
import tf2_py as tf2

rospy.init_node("publish_static_tfs")
tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)
br_static = tf2_ros.StaticTransformBroadcaster()
rospy.sleep(0.2)

tfs = []
# AoA cube
tf = TransformStamped()
tf.header.frame_id = "os_lidar"
tf.child_frame_id = "aoa_bottom_mount"
tf.header.stamp = rospy.Time.now()
tf.transform.translation = Vector3(0,0,0.095)
tf.transform.rotation = Quaternion(0,0,1,0)
tfs += [tf]
tf = TransformStamped()
tf.header.frame_id = "aoa_bottom_mount"
tf.child_frame_id = "aoa"
tf.header.stamp = rospy.Time.now()
tf.transform.translation = Vector3(0,0,0.064)
tf.transform.rotation = Quaternion(0,0,0,1)
tfs += [tf]
tf = TransformStamped()
tf.header.frame_id = "aoa_bottom_mount"
tf.child_frame_id = "aoa_top_mount"
tf.header.stamp = rospy.Time.now()
tf.transform.translation = Vector3(0,0,0.128)
tf.transform.rotation = Quaternion(0,0,0,1)
tfs += [tf]

# locator reference frame
tf = TransformStamped()
tf.header.frame_id = "middle_camera_mount"
tf.child_frame_id = "locator"
tf.header.stamp = rospy.Time.now()
tf.transform.translation = Vector3(0.03, 0, 0.047)
tf.transform.rotation = Quaternion(0,0,0,1)
tfs += [tf]

br_static.sendTransform(tfs)
rospy.spin()
