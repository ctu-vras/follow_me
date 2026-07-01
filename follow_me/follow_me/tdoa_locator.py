#!/usr/bin/env python3
# 3 TDoA anchors -> 1 anchor

import copy
import threading
from collections import deque, defaultdict

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from numpy.polynomial import Polynomial
from scipy.optimize import least_squares
from scipy.signal import lfilter, lfilter_zi, butter

from tf2_ros import Buffer, TransformListener
from uwb_tdoa_interfaces.msg import TDoAMeas
from xplraoa_ros_interfaces.msg import Angles
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

from follow_me.utils import get_angle, get_transform

C = 299792458
CUTOFF = 1.0
RANGE = 4.0
CALIB_LEN = 50
R_const = 100
Q_const = 1


class Kalman:
    def __init__(self, x0, P0, dt):
        # x = [x, y]
        # measurement = [x, y]
        # input = None
        self.x = x0
        self.x_cov = P0
        self.dt = dt
        self.A = np.array(
            [
                [1, 0],
                [0, 1],
            ]
        )
        self.Q = Q_const * np.eye(2)
        self.R = R_const * np.eye(2)
        self.H = np.eye(2)

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
        self.x_cov = np.matmul(np.eye(2) - np.matmul(K, self.H), self.x_cov)
        return self.x, self.x_cov


class Locator(Node):
    def __init__(self):
        super().__init__("tdoa_locator")
        self.msg_lock = threading.Lock()
        self.stamps = {}
        self.vars = {}
        self.frame_sn = {}

        # tf2
        self.declare_parameter("fixed_frame", Parameter.Type.STRING)
        self.declare_parameter("anchors.address", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("anchors.position", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("num_anchors", Parameter.Type.INTEGER)
        self.declare_parameter("target.anchor", Parameter.Type.STRING)
        self.declare_parameter("target.tags", Parameter.Type.STRING_ARRAY)
        # values mirrored from the /uwb/twr node parameters (supplied via launch)
        self.declare_parameter("twr_human_frame", Parameter.Type.STRING)
        self.declare_parameter("twr_ids", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("twr_positions", Parameter.Type.DOUBLE_ARRAY)

        self.fixed_frame = self.get_parameter("fixed_frame").value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # TDoA config
        self.anchors_address = self.get_parameter("anchors.address").value
        self.coords = self.get_parameter("anchors.position").value
        self.num_anchors = self.get_parameter("num_anchors").value
        self.positions = {}
        self.tof = {}
        self.lp_filter = butter(3, CUTOFF, fs=10.0)
        self.zi = {}
        self.calib_queue = defaultdict(dict)
        self.calib = {}
        for i in range(len(self.anchors_address)):
            address = self.anchors_address[i]
            self.positions[address] = np.array(
                [
                    [self.coords[3 * i]],
                    [self.coords[3 * i + 1]],
                    [self.coords[3 * i + 2]],
                ]
            )
            self.zi[address] = lfilter_zi(self.lp_filter[0], self.lp_filter[1])
            self.calib[address] = Polynomial([0.0, 1.0])
        self.anchors = list(self.positions.keys())

        # TWR config
        self.estimate_frame = self.get_parameter("twr_human_frame").value
        self.target_anchor = self.get_parameter("target.anchor").value
        self.target_tags = self.get_parameter("target.tags").value
        self.target_ids = self.get_parameter("twr_ids").value
        self.target_coords = self.get_parameter("twr_positions").value
        self.target_positions = {}
        self.subscribers = {}
        for i in range(len(self.target_ids)):
            id_l = self.target_ids[i].lower()
            self.target_positions[id_l] = np.array(
                [
                    [self.target_coords[3 * i]],
                    [self.target_coords[3 * i + 1]],
                    [self.target_coords[3 * i + 2]],
                ]
            )

        # rviz visualization
        self.marker = Marker()
        self.marker.header.frame_id = self.fixed_frame
        self.marker.type = self.marker.ARROW
        self.marker.action = self.marker.ADD
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.2
        self.marker.scale.z = 0.3
        self.marker.color.a = 1.0
        self.marker.color.r = 1.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 0.0

        # angle estimation
        self.filter = Kalman(np.array([[0], [0]]), np.eye(2), 0.1)
        self.initialised = False
        self.angle_pub = self.create_publisher(Angles, "angle", 1)
        self.last_estimate = None
        self.marker_pub = self.create_publisher(Marker, "heading", 1)
        self.tim2 = self.create_timer(0.1, self.get_estimate)

        self.subs1 = self.create_subscription(
            TDoAMeas, "measurements", self.tdoa_cb, 1
        )

        self.tim = self.create_timer(1.0, self.auto_calibrate)
        self.tim2 = self.create_timer(0.1, self.estimate_angle)

    def get_estimate(self):
        t = get_transform(self, self.tf_buffer, self.fixed_frame, self.estimate_frame)
        if t is not None:
            est = np.array([t[0, 3], t[1, 3]])
            if np.linalg.norm(est) != 0:
                est /= np.linalg.norm(est)
            self.last_estimate = est

    def tdoa_cb(self, msg):
        with self.msg_lock:
            tag = False
            stamps = {}
            for m in msg.measurements:
                if m.target == self.target_anchor:
                    if m.meas_valid:
                        self.stamps[m.address] = m.stamp
                        self.vars[m.address] = m.variance
                        self.frame_sn[m.address] = m.frame_sn
                else:
                    if m.meas_valid:
                        tag = True
                        stamps[m.address] = m.stamp
        if tag:
            for i in range(len(self.anchors)):
                id1 = self.anchors[i]
                id2 = self.anchors[(i + 1) % len(self.anchors)]
                d1 = np.linalg.norm(
                    self.positions[id1] - self.target_positions[m.target]
                )
                d2 = np.linalg.norm(
                    self.positions[id2] - self.target_positions[m.target]
                )
                expected_diff = d1 - d2
                diff = C * (stamps[id1] - stamps[id2])
                if m.target not in self.calib_queue[id1].keys():
                    self.calib_queue[id1][m.target] = deque()
                self.calib_queue[id1][m.target].append((expected_diff, diff))

    def auto_calibrate(self):
        for a in self.anchors:
            l = []
            for t in self.target_tags:
                if t not in self.calib_queue[a].keys():
                    continue
                while len(self.calib_queue[a][t]) >= CALIB_LEN:
                    self.calib_queue[a][t].popleft()
                if len(self.calib_queue[a][t]) != 0:
                    mean = np.mean(self.calib_queue[a][t], axis=0)
                    l += [(mean[1], mean[0])]
            l = np.array(l)
            if len(l) == 0:
                return
            elif len(l) == 1:
                cc = np.mean(l[:, 0] - l[:, 1])
                self.calib[a] = Polynomial([cc, 1.0])
            else:
                p = Polynomial.fit(l[:, 1], l[:, 0], 1)
                p = p.convert()
                self.calib[a] = p

    def intersectionPoint(self, guess, p_tdoa, d_tdoa, w):
        x_t1 = []
        y_t1 = []
        x_t2 = []
        y_t2 = []
        for i in range(len(p_tdoa)):
            x_t1 += [[p_tdoa[i][0][0, 0]]]
            y_t1 += [[p_tdoa[i][0][1, 0]]]
            x_t2 += [[p_tdoa[i][1][0, 0]]]
            y_t2 += [[p_tdoa[i][1][1, 0]]]
        x_t1_tdoa = np.array(x_t1)
        y_t1_tdoa = np.array(y_t1)
        x_t2_tdoa = np.array(x_t2)
        y_t2_tdoa = np.array(y_t2)
        d_tdoa = np.array(d_tdoa)

        last = self.last_estimate

        def eq(g):
            x, y = g

            f_tdoa = (
                np.sqrt((x - x_t1_tdoa) ** 2 + (y - y_t1_tdoa) ** 2)
                - np.sqrt((x - x_t2_tdoa) ** 2 + (y - y_t2_tdoa) ** 2)
                - d_tdoa
            ) ** 2
            f = np.vstack((f_tdoa, 10 * (np.linalg.norm(g) - 1)))
            return f.flatten().tolist()

        gu = [np.array([1, 0]), np.array([0, 1]), np.array([-1, 1]), np.array([0, -1])]
        best = None
        cost = np.inf
        for g in gu:
            ans = least_squares(eq, g, loss="soft_l1", verbose=0)
            if ans.success and ans.cost < cost:
                best = ans.x
                cost = ans.cost
        return best

    def estimate_angle(self):
        if len(self.stamps.keys()) != self.num_anchors:
            self.get_logger().warn("No data")
            return

        with self.msg_lock:
            stamps = copy.deepcopy(self.stamps)

        d = []
        pos = []
        w = []
        for i in range(len(self.anchors)):
            id1 = self.anchors[i]
            id2 = self.anchors[(i + 1) % len(self.anchors)]
            a1 = self.positions[id1]
            a2 = self.positions[id2]
            d_a2a = np.linalg.norm(a1 - a2)
            # diff = self.calib[id1](C * (stamps[id1] - stamps[id2]))
            # FIXME: faithful port of the ROS1 original, where `diff` is left
            # undefined here (the line above was commented out upstream).

            if diff > d_a2a:
                diff = d_a2a

            w += [[1 - np.abs(diff) / d_a2a]]

            var = C * (self.vars[id1] + self.vars[id2])
            diff_avg, self.zi[id1] = lfilter(
                self.lp_filter[0], self.lp_filter[1], np.array([diff]), zi=self.zi[id1]
            )
            diff_avg = float(diff_avg)

            d += [[diff_avg]]
            pos += [[a1, a2]]

        # solve NLS
        if self.last_estimate is None:
            guess = np.array([1.0, 0.0])
        else:
            guess = self.last_estimate
        est = self.intersectionPoint(guess, pos, d, w)
        if est is None:
            self.get_logger().warn("No solution")
            return

        if not self.initialised:
            self.filter.set_initial(np.array([[est[0]], [est[1]]]), np.eye(2))
            self.initialised = True
        else:
            self.filter.correct(np.array([[est[0]], [est[1]]]))
            est, est_cov = self.filter.predict()
            est = est.flatten()
        self.last_estimate = est
        Va = np.array([[1.0], [0.0], [0.0]])
        Vb = np.array([[est[0]], [est[1]], [0.0]])
        Vn = np.array([[0.0], [0.0], [1.0]])
        angle = get_angle(Va, Vb, Vn)
        msg = Angles(azimuth=float(angle), elevation=0.0, rssi=0.0)
        self.angle_pub.publish(msg)
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.points = [
            Point(x=0.0, y=0.0, z=0.0),
            Point(x=float(2 * np.cos(angle)), y=float(2 * np.sin(angle)), z=0.0),
        ]
        self.marker_pub.publish(self.marker)


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
