#!/usr/bin/env python3
# N UWB TWR tags -> 1 anchor + M BT anchors -> 1 BT tag

from collections import deque
from functools import partial

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from scipy.optimize import least_squares
from scipy.signal import lfilter, lfilter_zi, butter

from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from geometry_msgs.msg import TransformStamped, Point, PoseWithCovarianceStamped, PoseStamped
from dwm1001_ros_interfaces.msg import UWBMeas
from std_msgs.msg import Bool, String
from follow_me_interfaces.msg import PositionEstimate, HeadingEstimate

from follow_me.utils import (
    Kalman,
    attach_kalman_param_callback,
    declare_kalman_parameters,
    get_transform,
)

CUTOFF = 3.0
fs = 10

np.set_printoptions(precision=3)


class Locator(Node):
    def __init__(self):
        super().__init__("twr_locator_old")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.declare_parameter("fixed_frame", Parameter.Type.STRING)
        self.declare_parameter("human_frame", Parameter.Type.STRING)
        self.declare_parameter("use_3d", False)
        self.declare_parameter("use_aoa", False)
        self.declare_parameter("mount_height", 0.0)
        self.declare_parameter("max_msg_delay", 0.3)
        # values mirrored from the /uwb/twr node parameters (supplied via launch)
        self.declare_parameter("twr_ids", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("twr_positions", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("twr_calibration", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("twr_target", Parameter.Type.STRING)

        self.fixed_frame = self.get_parameter("fixed_frame").value
        self.human_frame = self.get_parameter("human_frame").value
        self.use_3d = self.get_parameter("use_3d").value
        self.use_aoa = self.get_parameter("use_aoa").value
        self.mount_height = self.get_parameter("mount_height").value
        self.max_msg_delay = self.get_parameter("max_msg_delay").value
        if not self.use_3d:
            self.get_logger().warn(
                "radio localisation is set to operate just in the x-y plane"
            )
        self.br = TransformBroadcaster(self)
        self.t = TransformStamped()
        self.t.header.frame_id = self.fixed_frame
        self.t.child_frame_id = self.human_frame
        self.t.transform.rotation.w = 1.0

        self.p = PoseWithCovarianceStamped()
        self.p.header.frame_id = self.fixed_frame
        self.p.pose.pose.orientation.w = 1.0

        # TWR
        self.ids = self.get_parameter("twr_ids").value
        self.coords = self.get_parameter("twr_positions").value
        self.coeffs = self.get_parameter("twr_calibration").value
        self.target = self.get_parameter("twr_target").value
        self.positions = {}
        self.calibration = {}
        self.subscribers = {}
        self.queues = {}
        self.ranges = {}
        self.ranges_avg = {}
        self.uwb_stamps = {}
        self.lp_filter = butter(3, CUTOFF, fs=fs)
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
            topic = "/uwb/twr/ID_" + id + "/distances"
            self.subscribers[id] = self.create_subscription(
                UWBMeas, topic, partial(self.range_cb, id=id), 1
            )
            self.zi[id] = lfilter_zi(self.lp_filter[0], self.lp_filter[1])

        # AoA
        self.heading_estimate = None
        self.heading_stamp = None
        if self.use_aoa:
            self.heading_subs = self.create_subscription(
                HeadingEstimate, "/bluetooth/aoa/angle", self.angle_cb, 1
            )

        if self.use_3d:
            self.last_pos = np.array([np.nan, np.nan, np.nan])
        else:
            self.last_pos = np.array([np.nan, np.nan])

        # r is set near the actual measurement noise of the TWR trilateration
        # solve (~0.5m std): the previous r=20 (~4.5m std) made every measurement
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
        self.pose_pub = self.create_publisher(PoseStamped, "estimate_pose", 1)
        self.pose_cov_pub = self.create_publisher(
            PoseWithCovarianceStamped, "estimate_pose_cov", 1
        )
        self.sound_pub = self.create_publisher(String, "/log_sound", 1)

        self.get_logger().info("Waiting for average value of the measurements")
        # warm up the filters before starting localisation (callbacks run while spinning)
        self.tim = None
        self._start_timer = self.create_timer(5.0, self._start)

    def _start(self):
        self._start_timer.cancel()
        self.get_logger().info("Starting radio localisation")
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

    def angle_cb(self, msg):
        self.heading_estimate = msg
        self.heading_stamp = self.now_sec()

    def intersectionPoint(self, guess, init, p_init):
        if self.use_aoa and self.heading_estimate is None:
            self.get_logger().warn("No estimate available")
            return None
        valid_ids = []
        x_t = []
        y_t = []
        z_t = []
        d = []
        age = []

        t = self.now_sec()
        for i in range(len(self.ids)):
            id = self.ids[i]
            meas_age = t - self.uwb_stamps[id]
            if meas_age > self.max_msg_delay:
                self.get_logger().error(
                    "Measurement from TWR tag %s is more than %.2f seconds old, disregarding it"
                    % (id, self.max_msg_delay)
                )
                id_mod = ""
                for j in range(len(id) - 1):
                    id_mod += id[j] + " "
                id_mod += id[-1]
                s = String(
                    data="Warning: Measurement from T W R tag %s is more than %.2f seconds old"
                    % (id_mod, self.max_msg_delay)
                )
                self.sound_pub.publish(s)
                continue
            valid_ids += [id]
            x_t += [[self.positions[id][0][0]]]
            y_t += [[self.positions[id][1][0]]]
            z_t += [[self.positions[id][2][0]]]
            d += [[self.ranges_avg[id]]]
            age += [meas_age]

        if len(valid_ids) < 3:
            self.get_logger().error(
                "Not enough up-to-date TWR measurements (%d) to solve for the position, "
                "at least 3 are required" % len(valid_ids)
            )
            return None

        use_aoa_now = False
        tf = None
        angles = None
        if self.use_aoa:
            heading_age = t - self.heading_stamp
            if heading_age > self.max_msg_delay:
                self.get_logger().warn(
                    "AoA heading estimate is more than %.2f seconds old, disregarding it"
                    % self.max_msg_delay
                )
            else:
                angles = [self.heading_estimate.azimuth, self.heading_estimate.elevation]
                angles_frame = self.heading_estimate.header.frame_id
                tf = get_transform(self, self.tf_buffer, angles_frame, self.fixed_frame)
                if tf is None:
                    self.get_logger().fatal(
                        "No transform between %s and %s" % (self.fixed_frame, angles_frame)
                    )
                    return None
                use_aoa_now = True

        x_t = np.array(x_t)
        y_t = np.array(y_t)
        z_t = np.array(z_t)
        d = np.array(d)

        w = self.weighting_function(d, valid_ids)

        # weight based on how old is the measurement
        w_t = np.zeros((len(valid_ids), 1))
        for i in range(len(valid_ids)):
            if age[i] < 1.0:
                w_t[i, 0] = min(age) / age[i]
        w_t = np.minimum(w_t, 0.5) # let the maximum difference in magnitude be 2 -> otherwise it could happen, that one measurement overtakes all just because of unlucky timing
        # # w_t = 0.01*np.ones((len(valid_ids), 1))

        print(f"age weights {w_t}")

        def eq(g):
            # TWR
            if self.use_3d:
                x, y, z = g
                f = (x - x_t) ** 2 + (y - y_t) ** 2 + (z - z_t) ** 2 - d**2
            else:
                x, y = g
                f = (
                    (x - x_t) ** 2
                    + (y - y_t) ** 2
                    + (self.mount_height - z_t) ** 2
                    - d**2
                )

            f = w_t * f  # weighting based on the age of the measurement

            if use_aoa_now:
                # AOA
                # perpendicular distance (in 2D) between the candidate point and the
                # ray originating at the AoA sensor with direction given by the
                # measured azimuth angle
                n = np.array([[np.cos(angles[0])], [np.sin(angles[0])]])
                p = np.matmul(tf, np.array([[x], [y], [0], [1]]))[:2, :]
                dist = float(n[0, 0] * p[1, 0] - n[1, 0] * p[0, 0])
                self.get_logger().info(f"aoa {angles[0]}\n aoa pose {n}\n pose {p}\n DIST {dist}")
                f = np.vstack((f, 3.0 * dist))

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
            self.get_logger().info(f"success: {ans.success}, {ans.status}, {ans.message}")
            self.get_logger().info(f"final residual (incl. AoA): {ans.fun} result pose {ans.x}")

            if ans.success:
                return ans.x
            else:
                return None

    def publish_pose(self):
        for id in self.ids:
            if np.isnan(self.ranges[id]):
                self.get_logger().warn("missing data")
                return

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
            self.pose_cov_pub.publish(self.p)

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

        pose = PoseStamped()
        pose.header.frame_id = self.fixed_frame
        pose.header.stamp = est.header.stamp
        pose.pose.position = Point(x=float(x[0]), y=float(x[1]), z=float(x[2]))
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)

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

    def weighting_function(self, d, ids):
        weights = []
        for i in range(len(ids)):
            id = ids[i]
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
