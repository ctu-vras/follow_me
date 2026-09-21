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

        # state used only to log notable events once, instead of on every message
        self._got_first_pose = False
        self._tf_failing = False
        self._stalled = False
        self._path_full = False

        self.pose_sub = self.create_subscription(
            PoseStamped, "pose", self.pose_cb, 10
        )
        self.path_pub = self.create_publisher(Path, "path", 1)
        self.pose_array_pub = self.create_publisher(PoseArray, "pose_array", 1)

        self.tim = self.create_timer(1.0 / publish_rate, self.publish_path)

        self.get_logger().info(
            f"Started: tracking 'pose' in frame '{self.map_frame}', publishing "
            f"'path' and 'pose_array' at {publish_rate} Hz (max_points={self.max_points}, "
            f"average_poses={self.average_poses}, tf_timeout={self.tf_timeout} s)"
        )

    def _param_cb(self, params):
        for p in params:
            if p.name == "max_points":
                self.max_points = p.value
                self._path_full = False
            elif p.name == "average_poses":
                self.average_poses = p.value
            elif p.name == "tf_timeout":
                self.tf_timeout = p.value
            elif p.name == "publish_rate":
                self.tim.cancel()
                self.tim = self.create_timer(1.0 / p.value, self.publish_path)
            elif p.name == "map_frame":
                self.get_logger().warning(
                    f"'map_frame' is only read at startup; ignoring runtime change to '{p.value}' "
                    f"(still using '{self.map_frame}')"
                )
                continue
            else:
                continue
            self.get_logger().info(f"Parameter '{p.name}' set to {p.value}")
        return SetParametersResult(successful=True)

    def _accept(self, pose):
        if not self._got_first_pose:
            self._got_first_pose = True
            self.get_logger().info(
                f"First pose received; tracking in frame '{self.map_frame}'"
            )
        self.window_poses.append(pose)

    def pose_cb(self, msg):
        if msg.header.frame_id == self.map_frame:
            self._accept(msg)
            return

        try:
            transformed = self.tf_buffer.transform(
                msg, self.map_frame, timeout=Duration(seconds=self.tf_timeout)
            )
        except (LookupException, ExtrapolationException) as ex:
            self._tf_failing = True
            self.get_logger().warning(
                f"Dropping pose: cannot transform '{msg.header.frame_id}' -> "
                f"'{self.map_frame}': {ex}",
                throttle_duration_sec=5.0,
            )
            return
        except ConnectivityException as ex:
            self._tf_failing = True
            self.get_logger().error(
                f"TF connectivity error transforming '{msg.header.frame_id}' -> "
                f"'{self.map_frame}': {ex}",
                throttle_duration_sec=5.0,
            )
            return

        if self._tf_failing:
            self._tf_failing = False
            self.get_logger().info(
                f"Transform '{msg.header.frame_id}' -> '{self.map_frame}' available again"
            )
        self._accept(transformed)

    def publish_path(self):
        if len(self.window_poses) == 0:
            if not self._got_first_pose:
                self.get_logger().info(
                    "Waiting for first pose on 'pose'...", throttle_duration_sec=10.0
                )
            elif not self._stalled:
                self._stalled = True
                self.get_logger().warning(
                    "No new poses since the last publish; path not updated"
                )
            return

        if self._stalled:
            self._stalled = False
            self.get_logger().info("Poses received again; resuming path updates")

        n_window = len(self.window_poses)
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
            if len(self.path.poses) > self.max_points and not self._path_full:
                self._path_full = True
                self.get_logger().info(
                    f"Path reached max_points={self.max_points}; dropping oldest points"
                )
            self.path.poses = self.path.poses[-self.max_points :]

        self.path.header.stamp = point.header.stamp
        self.path_pub.publish(self.path)

        pose_array = PoseArray()
        pose_array.header = self.path.header
        pose_array.poses = [p.pose for p in self.path.poses]
        self.pose_array_pub.publish(pose_array)

        self.get_logger().debug(
            f"Published {len(self.path.poses)} points "
            f"({'averaged' if self.average_poses else 'latest of'} {n_window} new poses)"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PoseTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"Shutting down with {len(node.path.poses)} points in path")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
