#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker


class CubeMarker(Node):
    def __init__(self):
        super().__init__("interactive_gripper_pose")
        self.pub = self.create_publisher(Marker, "aoa_cube", 1)

        self.marker = Marker()
        self.marker.type = Marker.MESH_RESOURCE
        self.marker.mesh_resource = "package://follow_me/meshes/aoa_cube.stl"
        self.marker.mesh_use_embedded_materials = True
        self.marker.header.frame_id = "aoa_bottom_mount"

        # Scale
        self.marker.scale.x = 0.001
        self.marker.scale.y = 0.001
        self.marker.scale.z = 0.001

        # Color
        self.marker.color.r = 0.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0
        self.marker.color.a = 0.0

        self.marker.pose.position.x = -0.065
        self.marker.pose.position.y = -0.065
        self.marker.pose.position.z = 0.0
        self.marker.pose.orientation.x = 0.0
        self.marker.pose.orientation.y = 0.0
        self.marker.pose.orientation.z = 0.0
        self.marker.pose.orientation.w = 1.0

        self.tim = self.create_timer(0.1, self.publish_marker)

    def publish_marker(self):
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.marker)


def main(args=None):
    rclpy.init(args=args)
    node = CubeMarker()
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
