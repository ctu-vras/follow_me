#!/usr/bin/env python

import rospy

from visualization_msgs.msg import *


rospy.init_node("interactive_gripper_pose")
pub = rospy.Publisher("aoa_cube", Marker, queue_size=1)

marker = Marker()
marker.type = Marker.MESH_RESOURCE
marker.mesh_resource = (
    "package://follow_me/meshes/aoa_cube.stl"
)
marker.mesh_use_embedded_materials = True
marker.header.frame_id = "aoa_bottom_mount"

# Scale
marker.scale.x = 0.001
marker.scale.y = 0.001
marker.scale.z = 0.001

# Color
marker.color.r = 0.0
marker.color.g = 0.0
marker.color.b = 0.0
marker.color.a = 0.0

marker.pose.position.x = -0.065
marker.pose.position.y = -0.065
marker.pose.position.z = 0.0
marker.pose.orientation.x = 0.0
marker.pose.orientation.y = 0.0
marker.pose.orientation.z = 0.0
marker.pose.orientation.w = 1.0

r = rospy.Rate(10)
while not rospy.is_shutdown():
    marker.header.stamp = rospy.Time.now()
    pub.publish(marker)
    r.sleep()