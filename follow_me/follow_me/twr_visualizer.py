#!/usr/bin/env python3

from functools import partial

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from dwm1001_ros_interfaces.msg import UWBMeas
from visualization_msgs.msg import Marker, MarkerArray


c = [
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0],
    [1.0, 1.0, 0.0],
    [0.0, 1.0, 1.0],
    [1.0, 0.0, 1.0],
]


class Visualizer(Node):
    def __init__(self):
        super().__init__("twr_visualizer")
        self.declare_parameter("fixed_frame", Parameter.Type.STRING)
        self.declare_parameter("ids", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("positions", Parameter.Type.DOUBLE_ARRAY)

        self.base_frame = self.get_parameter("fixed_frame").value
        self.br_static = StaticTransformBroadcaster(self)

        self.ids = self.get_parameter("ids").value
        self.coords = self.get_parameter("positions").value
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
            self.subscribers[id] = self.create_subscription(
                UWBMeas, topic, partial(self.range_cb, id=id), 1
            )
            self.colors[id] = c[i % len(c)]

            t2 = TransformStamped()
            t2.header.stamp = self.get_clock().now().to_msg()
            t2.header.frame_id = self.base_frame
            t2.child_frame_id = self.ids[i]
            t2.transform.translation.x = float(self.positions[id][0][0])
            t2.transform.translation.y = float(self.positions[id][1][0])
            t2.transform.translation.z = float(self.positions[id][2][0])
            t2.transform.rotation.w = 1.0
            self.tfs += [t2]

            marker = Marker()
            marker.header.frame_id = self.base_frame
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = float(self.colors[id][0])
            marker.color.g = float(self.colors[id][1])
            marker.color.b = float(self.colors[id][2])
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = float(self.positions[id][0][0])
            marker.pose.position.y = float(self.positions[id][1][0])
            marker.pose.position.z = float(self.positions[id][2][0])
            self.cubes += [marker]
        self.br_static.sendTransform(self.tfs)

        self.br = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(MarkerArray, "markers", 1)
        self.tim = self.create_timer(0.1, self.visualize)

    def range_cb(self, msg, id):
        if len(msg.measurements) != 0:
            self.ranges[id] = msg.measurements[0].dist

    def visualize(self):
        # plot to rviz
        markers = MarkerArray()
        for id in self.ids:
            if np.isnan(self.ranges[id]):
                continue

            marker = Marker()
            marker.header.frame_id = self.base_frame
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = float(2 * self.ranges[id])
            marker.scale.y = float(2 * self.ranges[id])
            marker.scale.z = float(2 * self.ranges[id])
            marker.color.a = 0.2
            marker.color.r = float(self.colors[id][0])
            marker.color.g = float(self.colors[id][1])
            marker.color.b = float(self.colors[id][2])
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = float(self.positions[id][0][0])
            marker.pose.position.y = float(self.positions[id][1][0])
            marker.pose.position.z = float(self.positions[id][2][0])

            markers.markers.append(marker)
        markers.markers += self.cubes
        id = 0
        for m in markers.markers:
            m.id = id
            id += 1
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = Visualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
