#!/usr/bin/env python3
"""Synthetic data generator for exercising radio_locator.py / aoa_locator.py
without real UWB TWR / Bluetooth AoA hardware.

Simulates a robot carrying 4 UWB TWR anchors and 1 Bluetooth AoA antenna,
following a scripted trajectory (straight - turn - straight - turn -
straight), and a single UWB/AoA tag moving in a straight line ~5-10 m ahead
of it. Publishes:

  - map -> base_link TF and a nav_msgs/Odometry (ground truth robot pose)
  - map -> <gt_tag_frame> TF (ground truth tag pose, for comparison in rviz)
  - base_link -> <imu_frame> TF (deliberately not axis-aligned with base_link,
    to exercise the rotate-into-fixed_frame handling in radio_locator.py)
  - dwm1001_ros_interfaces/UWBMeas on /uwb/twr/ID_<id>/distances per anchor
  - xplraoa_ros_interfaces/Angles on /bluetooth/aoa/<id>/angles_avg per antenna
  - sensor_msgs/Imu (gyro only) on /imu/data, from the robot's ground-truth
    yaw rate

UWB and AoA measurements are Gaussian-noised and computed from the ground
truth trajectory evaluated at (now - delay), to emulate real sensor latency.
"""

import math
from dataclasses import dataclass

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from scipy.spatial.transform import Rotation

from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Point, Quaternion, Vector3
from sensor_msgs.msg import Imu
from dwm1001_ros_interfaces.msg import UWBMeas, Anchor
from xplraoa_ros_interfaces.msg import Angles


@dataclass
class TwistSegment:
    duration: float
    v: float
    w: float


class UnicycleTrajectory:
    """Closed-form unicycle-model trajectory built from a list of constant
    (v, w) segments. Optionally loops (wraps back to the start pose, which
    causes a position jump at the wrap point since the scripted path is not
    closed)."""

    def __init__(self, segments, x0=0.0, y0=0.0, theta0=0.0):
        self.segments = segments
        self.x0, self.y0, self.theta0 = x0, y0, theta0
        self.total_duration = sum(s.duration for s in segments)

    def _clip(self, t, loop):
        if self.total_duration <= 0.0:
            return 0.0
        if loop:
            return t % self.total_duration
        return min(max(t, 0.0), self.total_duration)

    def pose_at(self, t, loop=True):
        t = self._clip(t, loop)
        x, y, theta = self.x0, self.y0, self.theta0
        remaining = t
        for seg in self.segments:
            dt = min(seg.duration, remaining)
            if dt <= 0.0:
                break
            if abs(seg.w) < 1e-9:
                x += seg.v * dt * math.cos(theta)
                y += seg.v * dt * math.sin(theta)
            else:
                r = seg.v / seg.w
                dtheta = seg.w * dt
                x += r * (math.sin(theta + dtheta) - math.sin(theta))
                y += r * (-math.cos(theta + dtheta) + math.cos(theta))
                theta += dtheta
            remaining -= dt
        return x, y, theta

    def velocity_at(self, t, loop=True):
        t = self._clip(t, loop)
        remaining = t
        for seg in self.segments:
            if remaining <= seg.duration or seg is self.segments[-1]:
                return seg.v, seg.w
            remaining -= seg.duration
        return 0.0, 0.0


def yaw_to_quat(theta):
    return Quaternion(
        x=0.0, y=0.0, z=float(math.sin(theta / 2.0)), w=float(math.cos(theta / 2.0))
    )


class SimDataGenerator(Node):
    def __init__(self):
        super().__init__("sim_data_generator")
        self._rng = np.random.default_rng()

        # ---- frames ----
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("gt_tag_frame", "tag_gt")
        self.declare_parameter("imu_frame", "imu_link")
        # roll, pitch, yaw (deg) of imu_frame relative to base_frame - non-zero
        # on purpose, to exercise radio_locator.py's imu->fixed_frame rotation
        # instead of silently working by accident on an aligned frame.
        self.declare_parameter("imu_rpy_deg", [15.0, -10.0, 40.0])
        self.declare_parameter("imu_offset", [0.0, 0.0, 0.0])

        # ---- shared hardware layout (mirrors twr.yaml / aoa.yaml schemas) ----
        self.declare_parameter("twr_ids", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("twr_positions", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("twr_target", Parameter.Type.STRING)
        self.declare_parameter("ids", Parameter.Type.STRING_ARRAY)  # AoA antenna ids
        self.declare_parameter(
            "positions", Parameter.Type.DOUBLE_ARRAY
        )  # AoA antenna offsets (xyz+quat per id)

        # ---- publish rates ----
        self.declare_parameter("gt_rate", 50.0)
        self.declare_parameter("uwb_rate", 10.0)
        self.declare_parameter("aoa_rate", 20.0)
        self.declare_parameter("imu_rate", 100.0)

        # ---- sensor latency + noise ----
        self.declare_parameter("uwb_delay", 0.05)
        self.declare_parameter("aoa_delay", 0.05)
        self.declare_parameter("imu_delay", 0.0)
        self.declare_parameter("uwb_noise_std", 0.06)
        self.declare_parameter("aoa_azimuth_noise_std", math.radians(2.0))
        self.declare_parameter("aoa_elevation_noise_std", math.radians(2.0))
        self.declare_parameter("aoa_rssi_noise_std", 3.0)
        self.declare_parameter("imu_gyro_noise_std", math.radians(0.5))

        # ---- trajectory shape ----
        self.declare_parameter("loop", True)
        self.declare_parameter("tag_gap", 7.0)
        self.declare_parameter("tag_speed", 0.45)
        self.declare_parameter("robot_straight_speed", 0.5)
        self.declare_parameter("robot_turn_speed", 0.3)
        self.declare_parameter("robot_straight_duration", 8.0)
        self.declare_parameter("robot_turn_duration", 1.5)
        self.declare_parameter(
            "robot_turn_angles_deg", [30.0, -60.0, 60.0, -60.0, 30.0]
        )

        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.gt_tag_frame = self.get_parameter("gt_tag_frame").value
        self.imu_frame = self.get_parameter("imu_frame").value
        self.imu_offset = np.array(self.get_parameter("imu_offset").value, dtype=float)
        self.imu_rot = Rotation.from_euler(
            "xyz", self.get_parameter("imu_rpy_deg").value, degrees=True
        )

        self.twr_ids = self.get_parameter("twr_ids").value
        twr_positions = self.get_parameter("twr_positions").value
        self.twr_target = self.get_parameter("twr_target").value
        self.aoa_ids = self.get_parameter("ids").value
        aoa_positions = self.get_parameter("positions").value

        self.gt_rate = self.get_parameter("gt_rate").value
        self.uwb_rate = self.get_parameter("uwb_rate").value
        self.aoa_rate = self.get_parameter("aoa_rate").value
        self.imu_rate = self.get_parameter("imu_rate").value

        self.uwb_delay = self.get_parameter("uwb_delay").value
        self.aoa_delay = self.get_parameter("aoa_delay").value
        self.imu_delay = self.get_parameter("imu_delay").value
        self.uwb_noise_std = self.get_parameter("uwb_noise_std").value
        self.aoa_az_noise_std = self.get_parameter("aoa_azimuth_noise_std").value
        self.aoa_el_noise_std = self.get_parameter("aoa_elevation_noise_std").value
        self.aoa_rssi_noise_std = self.get_parameter("aoa_rssi_noise_std").value
        self.imu_gyro_noise_std = self.get_parameter("imu_gyro_noise_std").value

        self.loop = self.get_parameter("loop").value

        if len(self.twr_ids) * 3 != len(twr_positions):
            raise ValueError("twr_positions must have exactly 3 values per twr_ids entry")
        if len(self.aoa_ids) * 7 != len(aoa_positions):
            raise ValueError("positions must have exactly 7 values per ids entry (aoa)")

        self.twr_offsets = {
            self.twr_ids[i]: np.array(twr_positions[3 * i : 3 * i + 3], dtype=float)
            for i in range(len(self.twr_ids))
        }
        self.aoa_offsets = {}
        for i in range(len(self.aoa_ids)):
            p = aoa_positions[7 * i : 7 * i + 7]
            self.aoa_offsets[self.aoa_ids[i]] = (
                np.array(p[0:3], dtype=float),
                Rotation.from_quat(p[3:7]),
            )

        # ---- trajectories ----
        straight_v = self.get_parameter("robot_straight_speed").value
        turn_v = self.get_parameter("robot_turn_speed").value
        straight_dur = self.get_parameter("robot_straight_duration").value
        turn_dur = self.get_parameter("robot_turn_duration").value
        turn_angles_deg = self.get_parameter("robot_turn_angles_deg").value

        # straight, then (turn, straight) for each angle in robot_turn_angles_deg
        robot_segments = [TwistSegment(straight_dur, straight_v, 0.0)]
        for angle_deg in turn_angles_deg:
            w_turn = math.radians(angle_deg) / turn_dur if turn_dur > 0 else 0.0
            robot_segments.append(TwistSegment(turn_dur, turn_v, w_turn))
            robot_segments.append(TwistSegment(straight_dur, straight_v, 0.0))
        self.robot_traj = UnicycleTrajectory(robot_segments, 0.0, 0.0, 0.0)

        tag_gap = self.get_parameter("tag_gap").value
        tag_speed = self.get_parameter("tag_speed").value
        # tag loops on the same period as the robot, so the pattern repeats in sync
        tag_segments = [TwistSegment(self.robot_traj.total_duration, tag_speed, 0.0)]
        self.tag_traj = UnicycleTrajectory(tag_segments, tag_gap, 0.0, 0.0)

        self.get_logger().info(
            "Simulated trajectory loop duration: %.1f s (loop=%s)"
            % (self.robot_traj.total_duration, self.loop)
        )

        # ---- publishers ----
        self.br = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, "odom_gt", 1)

        self.uwb_pubs = {
            id: self.create_publisher(UWBMeas, "/uwb/twr/ID_%s/distances" % id, 1)
            for id in self.twr_ids
        }
        self.aoa_pubs = {
            id: self.create_publisher(Angles, "/bluetooth/aoa/%s/angles_avg" % id, 1)
            for id in self.aoa_ids
        }
        self.imu_pub = self.create_publisher(Imu, "/imu/data", 1)

        self.start_time = self.get_clock().now()

        self.create_timer(1.0 / self.gt_rate, self.publish_ground_truth)
        self.create_timer(1.0 / self.uwb_rate, self.publish_uwb)
        self.create_timer(1.0 / self.aoa_rate, self.publish_aoa)
        self.create_timer(1.0 / self.imu_rate, self.publish_imu)

    def elapsed(self):
        return (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

    def robot_pose(self, t):
        return self.robot_traj.pose_at(t, self.loop)

    def robot_velocity(self, t):
        return self.robot_traj.velocity_at(t, self.loop)

    def tag_position(self, t):
        x, y, _ = self.tag_traj.pose_at(t, self.loop)
        return np.array([x, y, 0.0])

    def _robot_world_pose(self, t):
        x, y, theta = self.robot_pose(t)
        return np.array([x, y, 0.0]), Rotation.from_euler("z", theta)

    def publish_ground_truth(self):
        t = self.elapsed()
        x, y, theta = self.robot_pose(t)
        v, w = self.robot_velocity(t)
        stamp = self.get_clock().now().to_msg()

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.map_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation = Vector3(x=float(x), y=float(y), z=0.0)
        tf.transform.rotation = yaw_to_quat(theta)
        self.br.sendTransform(tf)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.map_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position = Point(x=float(x), y=float(y), z=0.0)
        odom.pose.pose.orientation = yaw_to_quat(theta)
        odom.twist.twist.linear.x = float(v)
        odom.twist.twist.angular.z = float(w)
        self.odom_pub.publish(odom)

        tag_x, tag_y, _ = self.tag_traj.pose_at(t, self.loop)
        tf_tag = TransformStamped()
        tf_tag.header.stamp = stamp
        tf_tag.header.frame_id = self.map_frame
        tf_tag.child_frame_id = self.gt_tag_frame
        tf_tag.transform.translation = Vector3(x=float(tag_x), y=float(tag_y), z=0.0)
        tf_tag.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.br.sendTransform(tf_tag)

        imu_q = self.imu_rot.as_quat()  # [x, y, z, w]
        tf_imu = TransformStamped()
        tf_imu.header.stamp = stamp
        tf_imu.header.frame_id = self.base_frame
        tf_imu.child_frame_id = self.imu_frame
        tf_imu.transform.translation = Vector3(
            x=float(self.imu_offset[0]),
            y=float(self.imu_offset[1]),
            z=float(self.imu_offset[2]),
        )
        tf_imu.transform.rotation = Quaternion(
            x=float(imu_q[0]), y=float(imu_q[1]), z=float(imu_q[2]), w=float(imu_q[3])
        )
        self.br.sendTransform(tf_imu)

    def publish_uwb(self):
        t_meas = self.elapsed() - self.uwb_delay
        robot_pos, robot_rot = self._robot_world_pose(t_meas)
        tag_pos = self.tag_position(t_meas)

        for id in self.twr_ids:
            anchor_pos = robot_pos + robot_rot.apply(self.twr_offsets[id])
            dist = float(np.linalg.norm(tag_pos - anchor_pos))
            dist += float(self._rng.normal(0.0, self.uwb_noise_std))

            msg = UWBMeas()
            msg.measurements = [Anchor(id=self.twr_target, location=Point(), dist=dist)]
            self.uwb_pubs[id].publish(msg)

    def publish_aoa(self):
        t_meas = self.elapsed() - self.aoa_delay
        robot_pos, robot_rot = self._robot_world_pose(t_meas)
        tag_pos = self.tag_position(t_meas)

        for id in self.aoa_ids:
            offset, local_rot = self.aoa_offsets[id]
            ant_pos = robot_pos + robot_rot.apply(offset)
            ant_rot = robot_rot * local_rot
            vec_local = ant_rot.inv().apply(tag_pos - ant_pos)

            azimuth = math.atan2(vec_local[1], vec_local[0])
            elevation = math.atan2(vec_local[2], math.hypot(vec_local[0], vec_local[1]))
            azimuth += float(self._rng.normal(0.0, self.aoa_az_noise_std))
            elevation += float(self._rng.normal(0.0, self.aoa_el_noise_std))

            dist = float(np.linalg.norm(tag_pos - ant_pos))
            rssi = -40.0 - 20.0 * math.log10(max(dist, 0.1))
            rssi += float(self._rng.normal(0.0, self.aoa_rssi_noise_std))

            msg = Angles(azimuth=azimuth, elevation=elevation, rssi=rssi)
            self.aoa_pubs[id].publish(msg)

    def publish_imu(self):
        t_meas = self.elapsed() - self.imu_delay
        _, w = self.robot_velocity(t_meas)
        w_noisy = w + float(self._rng.normal(0.0, self.imu_gyro_noise_std))
        # planar unicycle model: body angular velocity is yaw rate only,
        # expressed here in base_frame before rotating into the (deliberately
        # misaligned) imu frame
        w_body = np.array([0.0, 0.0, w_noisy])
        w_imu = self.imu_rot.inv().apply(w_body)

        gyro_var = self.imu_gyro_noise_std**2
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.imu_frame
        msg.angular_velocity = Vector3(x=float(w_imu[0]), y=float(w_imu[1]), z=float(w_imu[2]))
        msg.angular_velocity_covariance = [
            gyro_var, 0.0, 0.0,
            0.0, gyro_var, 0.0,
            0.0, 0.0, gyro_var,
        ]
        # orientation / linear acceleration not simulated - flag as unknown
        # per REP-145 (covariance[0] == -1)
        msg.orientation_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0
        self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimDataGenerator()
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
