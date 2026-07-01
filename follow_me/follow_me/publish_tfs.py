#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
from tf2_ros import StaticTransformBroadcaster


class StaticTfPublisher(Node):
    def __init__(self):
        super().__init__("publish_static_tfs")
        self.br_static = StaticTransformBroadcaster(self)

        tfs = []
        stamp = self.get_clock().now().to_msg()

        # AoA cube
        tf = TransformStamped()
        tf.header.frame_id = "base_link"
        tf.child_frame_id = "aoa"
        tf.header.stamp = stamp
        tf.transform.translation = Vector3(x=0.0, y=0.0, z=0.0)
        tf.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        tfs += [tf]

        # locator reference frame
        tf = TransformStamped()
        tf.header.frame_id = "base_link"
        tf.child_frame_id = "locator"
        tf.header.stamp = stamp
        tf.transform.translation = Vector3(x=0.0, y=0.0, z=0.0)
        tf.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        tfs += [tf]

        self.br_static.sendTransform(tfs)


def main(args=None):
    rclpy.init(args=args)
    node = StaticTfPublisher()
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
