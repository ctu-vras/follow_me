#!/usr/bin/env python3
# N UWB TWR tags -> 1 anchor

from collections import deque
from functools import partial

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from scipy.optimize import least_squares
from scipy.signal import lfilter, lfilter_zi, butter

from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Point, PoseWithCovarianceStamped
from dwm1001_ros_interfaces.msg import UWBMeas
from std_msgs.msg import Bool, String
from follow_me_interfaces.msg import PositionEstimate

from follow_me.utils import Kalman, attach_kalman_param_callback, declare_kalman_parameters

CUTOFF = 3.0

np.set_printoptions(precision=3)


class Locator(Node):
    def __init__(self):
        super().__init__("twr_locator")

        self.declare_parameter("fixed_frame", Parameter.Type.STRING)
        self.declare_parameter("human_frame", Parameter.Type.STRING)
        self.declare_parameter("use_3d", False)
        self.declare_parameter("mount_height", 0.0)
        self.declare_parameter("target", Parameter.Type.STRING)
        self.declare_parameter("ids", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("positions", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("calibration", Parameter.Type.DOUBLE_ARRAY)

        self.fixed_frame = self.get_parameter("fixed_frame").value
        self.human_frame = self.get_parameter("human_frame").value
        self.use_3d = self.get_parameter("use_3d").value
        self.mount_height = self.get_parameter("mount_height").value
        self.target = self.get_parameter("target").value
        if not self.use_3d:
            self.get_logger().warn(
                "TWR localisation is set to operate just in the x-y plane"
            )
        self.br = TransformBroadcaster(self)
        self.t = TransformStamped()
        self.t.header.frame_id = self.fixed_frame
        self.t.child_frame_id = self.human_frame
        self.t.transform.rotation.w = 1.0

        self.p = PoseWithCovarianceStamped()
        self.p.header.frame_id = self.fixed_frame
        self.p.pose.pose.orientation.w = 1.0

        self.ids = self.get_parameter("ids").value
        self.coords = self.get_parameter("positions").value
        self.coeffs = self.get_parameter("calibration").value
        self.positions = {}
        self.calibration = {}
        self.subscribers = {}
        self.queues = {}
        self.ranges = {}
        self.ranges_avg = {}
        self.uwb_stamps = {}
        self.lp_filter = butter(3, CUTOFF / (2 * np.pi))
        self.zi = {}
        for i in range(len(self.ids)):
            id = self.ids[i]
            self.positions[id] = np.array(
                [
                    [self.coords[3 * i]],
                    [self.coords[3 * i + 1]],
                    [self.coords[3 * i + 2]],
                ]
            )
            self.calibration[id] = [
                self.coeffs[4 * i],
                self.coeffs[4 * i + 1],
                self.coeffs[4 * i + 2],
                self.coeffs[4 * i + 3],
            ]
            self.queues[id] = deque()
            self.ranges[id] = np.nan
            self.ranges_avg[id] = 0.0
            self.uwb_stamps[id] = None
            topic = "ID_" + id + "/distances"
            self.subscribers[id] = self.create_subscription(
                UWBMeas, topic, partial(self.range_cb, id=id), 1
            )
            self.zi[id] = lfilter_zi(self.lp_filter[0], self.lp_filter[1])

        if self.use_3d:
            self.last_pos = np.array([np.nan, np.nan, np.nan])
        else:
            self.last_pos = np.array([np.nan, np.nan])

        # r is set near the actual measurement noise of the TWR trilateration
        # solve (~0.5m std): the previous r=50 (~7m std) made every measurement
        # look "expected" no matter how far off, which (combined with the
        # predict/correct ordering bug) is why the innovation gate below would
        # never have fired - a real jump and normal noise looked statistically
        # identical. gate_threshold/max_inflation control how aggressively real
        # jumps (e.g. the robot turning suddenly) get absorbed in ~1-2 cycles
        # instead of smoothed away over seconds; q is the nominal per-second
        # process noise for calm conditions.
        kalman_params = declare_kalman_parameters(self, q=1.0, r=0.25)
        self.filter = Kalman(np.array([[0.0], [0.0], [0.0]]), np.eye(3), **kalman_params)
        attach_kalman_param_callback(self, self.filter)
        self.filter_last_time = None
        self.initialised = False

        self.started = False
        self.pub = self.create_publisher(Bool, "/detection_ready", 1)
        self.estimate_pub = self.create_publisher(PositionEstimate, "estimate", 1)
        self.pub2 = self.create_publisher(
            PoseWithCovarianceStamped, "pose_cov", 1
        )
        self.sound_pub = self.create_publisher(String, "/log_sound", 1)

        self.get_logger().info("Waiting for average value of the range measurements")
        # warm up the filters before starting localisation (callbacks run while spinning)
        self.tim = None
        self._start_timer = self.create_timer(5.0, self._start)

    def _start(self):
        self._start_timer.cancel()
        self.get_logger().info("Starting TWR localisation")
        self.tim = self.create_timer(0.1, self.publish_pose)

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def range_cb(self, msg, id):
        for m in msg.measurements:
            if m.id != self.target:
                continue
            d = m.dist
            self.ranges[id] = d
            p = self.calibration[id]
            d_cal = p[0] * d**3 + p[1] * d**2 + p[2] * d + p[3]
            d_filt, zi = lfilter(
                self.lp_filter[0], self.lp_filter[1], np.array([d_cal]), zi=self.zi[id]
            )
            self.zi[id] = zi
            self.ranges[id] = d
            self.ranges_avg[id] = float(d_filt)
            self.uwb_stamps[id] = self.now_sec()

    def intersectionPoint(self, guess, init, p_init):
        x_t = []
        y_t = []
        z_t = []
        d = []
        age = []

        t = self.now_sec()
        for i in range(len(self.ids)):
            id = self.ids[i]
            x_t += [[self.positions[id][0][0]]]
            y_t += [[self.positions[id][1][0]]]
            z_t += [[self.positions[id][2][0]]]
            d += [[self.ranges_avg[id]]]
            age += [t - self.uwb_stamps[id]]
            if age[-1] > 0.3:
                self.get_logger().error(
                    "Measurement from TWR tag %s is more than 0.3 seconds old" % (id)
                )
                id_mod = ""
                for i in range(len(id) - 1):
                    id_mod += id[i] + " "
                id_mod += id[-1]
                s = String(
                    data="Warning: Measurement from T W R tag %s is more than 0.3 seconds old"
                    % (id_mod)
                )
                self.sound_pub.publish(s)

        x_t = np.array(x_t)
        y_t = np.array(y_t)
        z_t = np.array(z_t)
        d = np.array(d)

        w = self.weighting_function(d)

        # weight based on how old is the measurement
        w_t = np.zeros((len(self.ids), 1))
        for i in range(len(self.ids)):
            if age[i] < 1.0:
                w_t[i, 0] = min(age) / age[i]

        def eq(g):
            if self.use_3d:
                x, y, z = g
                f = (x - x_t) ** 2 + (y - y_t) ** 2 + (z - z_t) ** 2 - d**2
                f_prev = (x - guess[0]) ** 2 + (y - guess[1]) ** 2 + (z - guess[2]) ** 2
            else:
                x, y = g
                f = (
                    (x - x_t) ** 2
                    + (y - y_t) ** 2
                    + (self.mount_height - z_t) ** 2
                    - d**2
                )
                f_prev = (x - guess[0]) ** 2 + (y - guess[1]) ** 2

            f = w_t * f

            return f.flatten().tolist()

        if init:
            best = None
            cost = np.inf
            for p in p_init:
                ans = least_squares(eq, p, loss="soft_l1", verbose=0)
                if ans.success and ans.cost < cost:
                    best = ans.x
                    cost = ans.cost
            return best
        else:
            ans = least_squares(eq, guess, loss="soft_l1", verbose=0)

            if ans.success:
                return ans.x
            else:
                return None

    def publish_pose(self):
        for id in self.ids:
            if np.isnan(self.ranges[id]):
                self.get_logger().warn("missing data")
                return

        if not self.started:
            # send ready signal to convoy node
            self.pub.publish(Bool(data=True))
            self.started = True

        # find the intersection point
        init = False
        p_init = []
        if np.any(np.isnan(self.last_pos)):
            p = self.positions[self.ids[0]]
            if self.use_3d:
                p_init += [
                    np.array([p[0][0] + self.ranges_avg[self.ids[0]], p[1][0], p[2][0]])
                ]
                p_init += [
                    np.array([p[0][0], p[1][0] + self.ranges_avg[self.ids[0]], p[2][0]])
                ]
                p_init += [
                    np.array([p[0][0] - self.ranges_avg[self.ids[0]], p[1][0], p[2][0]])
                ]
                p_init += [
                    np.array([p[0][0], p[1][0] - self.ranges_avg[self.ids[0]], p[2][0]])
                ]
            else:
                p_init += [np.array([p[0][0] + self.ranges_avg[self.ids[0]], p[1][0]])]
                p_init += [np.array([p[0][0], p[1][0] + self.ranges_avg[self.ids[0]]])]
                p_init += [np.array([p[0][0] - self.ranges_avg[self.ids[0]], p[1][0]])]
                p_init += [np.array([p[0][0], p[1][0] - self.ranges_avg[self.ids[0]]])]
            init = True
        x = self.intersectionPoint(self.last_pos, init, p_init)
        if x is None:
            self.get_logger().warn("intersection point not found")
            return
        if not self.use_3d:
            x = np.concatenate((x, np.array([self.mount_height])))

        now = self.now_sec()
        if not self.initialised:
            self.filter.set_initial(np.array([[x[0]], [x[1]], [x[2]]]), np.eye(3))
            self.initialised = True
            self.filter_last_time = now
        else:
            dt = now - self.filter_last_time
            self.filter_last_time = now
            x_new, cov = self.filter.step(np.array([[x[0]], [x[1]], [x[2]]]), dt)
            cov_pose = np.zeros((6, 6))
            cov_pose[:3, :3] = cov[:3, :3]
            x = x_new[0:3, :].flatten()

            self.p.header.stamp = self.get_clock().now().to_msg()
            self.p.pose.pose.position.x = float(x[0])
            self.p.pose.pose.position.y = float(x[1])
            self.p.pose.pose.position.z = float(x[2])

            self.p.pose.covariance = cov_pose.flatten().tolist()

            self.pub2.publish(self.p)
        if self.use_3d:
            self.last_pos = x
        else:
            self.last_pos = x[:2]

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

    def weighting_function(self, d):
        weights = []
        for i in range(len(self.ids)):
            id = self.ids[i]
            d_meas = d[i]
            pos = self.positions[id]
            if not self.use_3d:
                pos = pos[:2, :]
            d_pred = np.linalg.norm(pos - self.last_pos[:, None])
            w = 1 / (100 * (d_meas - d_pred) ** 2 + 1e-4)
            weights += [w]
        weights = np.array(weights)  # (n_uwbs, 1)
        return weights


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
