#!/usr/bin/env python3
# 1 UWB TWR tags -> 1 anchor + 4 BT anchors -> 1 BT tag

from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.signal import lfilter, lfilter_zi, butter

from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from geometry_msgs.msg import TransformStamped, Point
from dwm1001_ros_interfaces.msg import UWBMeas
from std_msgs.msg import Bool, String
from follow_me_interfaces.msg import PositionEstimate, HeadingEstimate

from follow_me.utils import get_transform

R_const = 2
Q_const = 1
CUTOFF = 3.0

np.set_printoptions(precision=3)


class Kalman:
    def __init__(self, x0, P0, dt):
        self.x = x0
        self.x_cov = P0
        self.dt = dt
        self.A = np.array(
            [
                [1, 0, 0],
                [0, 1, 0],
                [0, 0, 1],
            ]
        )
        self.Q = Q_const * np.eye(3)
        self.R = R_const * np.eye(3)
        self.H = np.array(
            [
                [1, 0, 0],
                [0, 1, 0],
                [0, 0, 1],
            ]
        )

    def set_initial(self, x0, P0):
        self.x = x0
        self.x_cov = P0

    def predict(self):
        self.x = np.matmul(self.A, self.x)
        self.x_cov = np.matmul(np.matmul(self.A, self.x_cov), self.A.T) + self.Q
        return self.x, self.x_cov

    def correct(self, measurement):
        K = np.matmul(
            np.matmul(self.x_cov, self.H.T),
            np.linalg.inv(np.matmul(np.matmul(self.H, self.x_cov), self.H.T) + self.R),
        )
        self.x = self.x + np.matmul(K, measurement - np.matmul(self.H, self.x))
        self.x_cov = np.matmul(
            np.eye(self.x_cov.shape[0]) - np.matmul(K, self.H), self.x_cov
        )
        return self.x, self.x_cov


class Locator(Node):
    def __init__(self):
        super().__init__("twr_locator")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.fixed_frame = "aoa_top_mount"
        self.human_frame = "position_cube"
        self.br = TransformBroadcaster(self)
        self.t = TransformStamped()
        self.t.header.frame_id = self.fixed_frame
        self.t.child_frame_id = self.human_frame
        self.t.transform.rotation.w = 1.0

        # TWR
        self.id = "C694"
        self.position = np.array([[0], [0.085], [-0.01]])
        self.calibration = {}
        self.queue = deque()
        self.range = np.nan
        self.range_avg = 0.0
        self.uwb_stamp = None
        self.lp_filter = butter(3, CUTOFF / (2 * np.pi))
        self.zi = lfilter_zi(self.lp_filter[0], self.lp_filter[1])
        topic = "/uwb/twr/ID_" + self.id + "/distances"
        self.twr_subscriber = self.create_subscription(
            UWBMeas, topic, self.range_cb, 1
        )

        # AoA
        self.heading_estimate = None
        self.heading_subs = self.create_subscription(
            HeadingEstimate, "/bluetooth/aoa/angle", self.angle_cb, 1
        )

        self.last_pos = np.array([np.nan, np.nan])

        self.filter = Kalman(np.array([[0], [0], [0], [0], [0]]), np.eye(5), 0.1)
        self.initialised = False

        self.started = False
        self.pub = self.create_publisher(Bool, "/detection_ready", 1)
        self.estimate_pub = self.create_publisher(PositionEstimate, "estimate", 1)
        self.sound_pub = self.create_publisher(String, "/log_sound", 1)

        self.get_logger().info("Waiting for average value of the measurements")
        # warm up the filter before starting localisation (callbacks run while spinning)
        self.tim = None
        self._start_timer = self.create_timer(5.0, self._start)

    def _start(self):
        self._start_timer.cancel()
        self.get_logger().info("Starting radio localisation")
        self.tim = self.create_timer(0.1, self.publish_pose)

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def range_cb(self, msg):
        if len(msg.measurements) != 0:
            d = msg.measurements[0].dist
            self.range = d
            d_cal = d
            d_filt, zi = lfilter(
                self.lp_filter[0], self.lp_filter[1], np.array([d_cal]), zi=self.zi
            )
            self.zi = zi
            self.range = d
            self.range_avg = float(d_filt)
            self.uwb_stamp = self.now_sec()

    def angle_cb(self, msg):
        self.heading_estimate = msg

    def intersectionPoint(self):
        if self.heading_estimate is None or np.isnan(self.range):
            self.get_logger().warn("No data available yet")
            return None

        (az, el) = [self.heading_estimate.azimuth, self.heading_estimate.elevation]
        angles_frame = self.heading_estimate.header.frame_id
        tf = get_transform(self, self.tf_buffer, self.fixed_frame, angles_frame)
        if tf is None:
            self.get_logger().fatal(
                "No transform between %s and %s" % (self.fixed_frame, angles_frame)
            )
            return None

        d = np.array(
            [[np.cos(el) * np.cos(az)], [np.cos(el) * np.sin(az)], [np.sin(el)]]
        )
        d /= np.linalg.norm(d)

        (a, b, c) = self.position[:, 0]
        (x0, y0, z0) = tf[:3, 3]
        (x1, y1, z1) = d[:, 0]
        r = self.range_avg

        A = x1**2 + y1**2 + z1**2
        B = (
            -2 * a * x1
            - 2 * b * y1
            - 2 * c * z1
            + 2 * x0 * x1
            + 2 * y0 * y1
            + 2 * z0 * z1
        )
        C = (
            a**2
            - 2 * a * x0
            + b**2
            - 2 * b * y0
            + c**2
            - 2 * c * z0
            + x0**2
            + y0**2
            + z0**2
            - r**2
        )

        roots = np.roots([A, B, C])
        t = None
        for root in roots:
            if np.isreal(root) and root >= 0:
                t = root
                break  # only one correct solution exists
        if t is None:
            return None
        else:
            x = np.array([x0 + t * x1, y0 + t * y1, z0 + t * z1])
            return x

    def publish_pose(self):
        # find the intersection point
        x = self.intersectionPoint()
        if x is None:
            self.get_logger().warn("intersection point not found")
            return

        if not self.initialised:
            self.filter.set_initial(np.array([[x[0]], [x[1]], [x[2]]]), np.eye(3))
            self.initialised = True
        else:
            self.filter.correct(np.array([[x[0]], [x[1]], [x[2]]]))
            x_new, cov = self.filter.predict()
            cov_pose = np.zeros((6, 6))
            cov_pose[:3, :3] = cov[:3, :3]
            x = x_new[0:3, :].flatten()

        # send estimate
        est = PositionEstimate()
        est.header.frame_id = self.fixed_frame
        est.header.stamp = self.get_clock().now().to_msg()
        est.position_estimate = Point(x=float(x[0]), y=float(x[1]), z=float(x[2]))
        self.estimate_pub.publish(est)

        # send tf
        self.t.header.stamp = self.get_clock().now().to_msg()
        self.t.transform.translation.x = float(x[0])
        self.t.transform.translation.y = float(x[1])
        self.t.transform.translation.z = float(x[2])
        self.br.sendTransform(self.t)

        # send ready signal
        if not self.started:
            self.pub.publish(Bool(data=True))
            self.started = True


def main(args=None):
    rclpy.init(args=args)
    node = Locator()
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
