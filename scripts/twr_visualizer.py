#!/usr/bin/env python
import rospy

import tf2_ros
from geometry_msgs.msg import TransformStamped, Point
from dwm1001c_ros.msg import UWBMeas, Anchor
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

import numpy as np


c = [
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0],
    [1.0, 1.0, 0.0],
    [0.0, 1.0, 1.0],
    [1.0, 0.0, 1.0],
]


class Visualizer:
    def __init__(self):
        self.base_frame = rospy.get_param("fixed_frame")
        self.br_static = tf2_ros.StaticTransformBroadcaster()
        rospy.sleep(0.5)

        self.ids = rospy.get_param("ids")
        self.coords = rospy.get_param("positions")
        self.positions = {}
        self.subscribers = {}
        self.ranges = {}
        self.colors = {}
        self.tfs = []
        self.cubes = []
        for i in range(len(self.ids)):
            id = self.ids[i]
            self.positions[id] = np.array(
                [
                    [self.coords[3 * i]],
                    [self.coords[3 * i + 1]],
                    [self.coords[3 * i + 2]],
                ]
            )
            self.ranges[id] = np.nan
            topic = "ID_" + id + "/distances"
            self.subscribers[id] = rospy.Subscriber(
                topic,
                UWBMeas,
                self.range_cb,
                id,
                queue_size=1,
            )
            self.colors[id] = c[i % len(c)]

            t2 = TransformStamped()
            t2.header.stamp = rospy.Time.now()
            t2.header.frame_id = self.base_frame
            t2.child_frame_id = self.ids[i]
            t2.transform.translation.x = self.positions[id][0][0]
            t2.transform.translation.y = self.positions[id][1][0]
            t2.transform.translation.z = self.positions[id][2][0]
            t2.transform.rotation.w = 1.0
            self.tfs += [t2]

            marker = Marker()
            marker.header.frame_id = self.base_frame
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1
            marker.color.r = self.colors[id][0]
            marker.color.g = self.colors[id][1]
            marker.color.b = self.colors[id][2]
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = self.positions[id][0][0]
            marker.pose.position.y = self.positions[id][1][0]
            marker.pose.position.z = self.positions[id][2][0]
            self.cubes += [marker]
        self.br_static.sendTransform(self.tfs)

        self.br = tf2_ros.TransformBroadcaster()
        self.marker_pub = rospy.Publisher("markers", MarkerArray, queue_size=1)
        self.tim = rospy.Timer(rospy.Duration(0.1), self.visualize)
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        self.tim.shutdown()
        rospy.sleep(0.5)

    def range_cb(self, msg, id):
        if len(msg.measurements) != 0:
            self.ranges[id] = msg.measurements[0].dist

    def visualize(self, _):
        # plot to rviz
        markers = MarkerArray()
        for id in self.ids:
            if np.isnan(self.ranges[id]):
                continue

            marker = Marker()
            marker.header.frame_id = self.base_frame
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 2 * self.ranges[id]
            marker.scale.y = 2 * self.ranges[id]
            marker.scale.z = 2 * self.ranges[id]
            marker.color.a = 0.2
            marker.color.r = self.colors[id][0]
            marker.color.g = self.colors[id][1]
            marker.color.b = self.colors[id][2]
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = self.positions[id][0][0]
            marker.pose.position.y = self.positions[id][1][0]
            marker.pose.position.z = self.positions[id][2][0]

            markers.markers.append(marker)
        markers.markers += self.cubes
        id = 0
        for m in markers.markers:
            m.id = id
            id += 1
        self.marker_pub.publish(markers)


if __name__ == "__main__":
    rospy.init_node("twr_visualizer")
    ms = Visualizer()
    rospy.spin()
