#!/usr/bin/env python3
# subscribes to a PoseStamped, transforms it into map_frame immediately, and
# tracks it as a nav_msgs Path (plus a matching PoseArray) at a lower, fixed
# publish rate

import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401  (registers PoseStamped transform support)
from rcl_interfaces.msg import SetParametersResult
from rclpy.duration import Duration
from rclpy.node import Node

from tf2_ros import Buffer, TransformListener, ConnectivityException, ExtrapolationException, LookupException
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import Path


def average_poses(poses):
    """average position arithmetically and orientation via a normalized
    quaternion sum (a good approximation for quaternions that are already
    close together, as is the case within one short averaging window)

    :param poses: list of geometry_msgs PoseStamped, all already in the same frame
    :return: geometry_msgs Pose
    """
    positions = np.array(
        [[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in poses]
    )
    pos_avg = positions.mean(axis=0)

    quats = np.array(
        [
            [
                p.pose.orientation.x,
                p.pose.orientation.y,
                p.pose.orientation.z,
                p.pose.orientation.w,
            ]
            for p in poses
        ]
    )
    q0 = quats[0]
    # flip antipodal quaternions (q and -q represent the same rotation) into
    # the same hemisphere as q0 before summing, otherwise they could cancel out
    signs = np.sign(np.sum(quats * q0, axis=1))
    signs[signs == 0] = 1.0
    quats = quats * signs[:, None]
    quat_sum = quats.sum(axis=0)
    quat_avg = quat_sum / np.linalg.norm(quat_sum)

    out = Pose()
    out.position.x, out.position.y, out.position.z = pos_avg.tolist()
    out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w = quat_avg.tolist()
    return out


class PoseTracker(Node):
    def __init__(self):
        super().__init__("pose_tracker")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("max_points", 100)
        self.declare_parameter("average_poses", False)
        self.declare_parameter("tf_timeout", 0.1)

        self.map_frame = self.get_parameter("map_frame").value
        self.max_points = self.get_parameter("max_points").value
        self.average_poses = self.get_parameter("average_poses").value
        self.tf_timeout = self.get_parameter("tf_timeout").value
        publish_rate = self.get_parameter("publish_rate").value

        self.add_on_set_parameters_callback(self._param_cb)

        self.path = Path()
        self.path.header.frame_id = self.map_frame

        self.window_poses = []

        self.pose_sub = self.create_subscription(
            PoseStamped, "pose", self.pose_cb, 10
        )
        self.path_pub = self.create_publisher(Path, "path", 1)
        self.pose_array_pub = self.create_publisher(PoseArray, "pose_array", 1)

        self.tim = self.create_timer(1.0 / publish_rate, self.publish_path)

    def _param_cb(self, params):
        for p in params:
            if p.name == "max_points":
                self.max_points = p.value
            elif p.name == "average_poses":
                self.average_poses = p.value
            elif p.name == "tf_timeout":
                self.tf_timeout = p.value
            elif p.name == "publish_rate":
                self.tim.cancel()
                self.tim = self.create_timer(1.0 / p.value, self.publish_path)
        return SetParametersResult(successful=True)

    def pose_cb(self, msg):
        if msg.header.frame_id == self.map_frame:
            self.window_poses.append(msg)
            return

        try:
            transformed = self.tf_buffer.transform(
                msg, self.map_frame, timeout=Duration(seconds=self.tf_timeout)
            )
        except (LookupException, ExtrapolationException):
            return
        except ConnectivityException as ex:
            self.get_logger().error(str(ex))
            return

        self.window_poses.append(transformed)

    def publish_path(self):
        if len(self.window_poses) == 0:
            return

        if self.average_poses:
            pose = average_poses(self.window_poses)
        else:
            pose = self.window_poses[-1].pose
        self.window_poses = []

        point = PoseStamped()
        point.header.stamp = self.get_clock().now().to_msg()
        point.header.frame_id = self.map_frame
        point.pose = pose

        self.path.poses.append(point)
        if self.max_points > 0:
            self.path.poses = self.path.poses[-self.max_points :]

        self.path.header.stamp = point.header.stamp
        self.path_pub.publish(self.path)

        pose_array = PoseArray()
        pose_array.header = self.path.header
        pose_array.poses = [p.pose for p in self.path.poses]
        self.pose_array_pub.publish(pose_array)


def main(args=None):
    rclpy.init(args=args)
    node = PoseTracker()
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
