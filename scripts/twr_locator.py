#!/usr/bin/env python
# N UWB TWR tags -> 1 anchor

import rospy

import tf2_ros
from geometry_msgs.msg import (
    TransformStamped,
    Point,
    PoseWithCovarianceStamped,
)
from dwm1001_ros.msg import UWBMeas
from std_msgs.msg import Bool, String
from follow_me.msg import PositionEstimate

import numpy as np
from scipy.optimize import least_squares
from scipy.signal import lfilter, lfilter_zi, butter
from collections import deque

R_const = 70
Q_const = 1
CUTOFF = 4.0

np.set_printoptions(precision=3)


class Kalman:
    def __init__(self, x0, P0, dt):
        # x = [x, y, z, v_x, v_y, a_x, a_y]
        # measurement = [x, y]
        # input = [v_x, v_y]
        self.x = x0
        self.x_cov = P0
        self.dt = dt
        # self.A = np.array(
        #     [
        #         [1, 0, 0, dt, 0, dt**2 / 2, 0],
        #         [0, 1, 0, 0, dt, 0, dt**2 / 2],
        #         [0, 0, 1, 0, 0, 0, 0],
        #         [0, 0, 0, 1, 0, dt, 0],
        #         [0, 0, 0, 0, 1, 0, dt],
        #         [0, 0, 0, 0, 0, 1, 0],
        #         [0, 0, 0, 0, 0, 0, 1],
        #     ]
        # )
        # self.A = np.array(
        #     [
        #         [1, 0, 0, dt, 0],
        #         [0, 1, 0, 0, dt],
        #         [0, 0, 1, 0, 0],
        #         [0, 0, 0, 1, 0],
        #         [0, 0, 0, 0, 1],
        #     ]
        # )
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

        # self.x[3:5,:] = np.clip(self.x[3:5,:], -0.05, 0.05)
        # self.x[5:7,:] = np.clip(self.x[5:7,:], -0.1, 0.1)
        # print(self.x[3:7,:].T)
        # print(self.x_cov)

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

        # self.x[3:5,:] = np.clip(self.x[3:5,:], -0.05, 0.05)
        # self.x[5:7,:] = np.clip(self.x[5:7,:], -0.1, 0.1)

        return self.x, self.x_cov


class Locator:
    def __init__(self):
        rospy.init_node("twr_locator")
        self.fixed_frame = rospy.get_param("fixed_frame")
        self.human_frame = rospy.get_param("human_frame")
        self.use_3d = rospy.get_param("use_3d")
        self.mount_height = rospy.get_param("mount_height")
        self.target = rospy.get_param("target")
        if not self.use_3d:
            rospy.logwarn("TWR localisation is set to operate just in the x-y plane")
        self.br = tf2_ros.TransformBroadcaster()
        self.t = TransformStamped()
        self.t.header.frame_id = self.fixed_frame
        self.t.child_frame_id = self.human_frame
        self.t.transform.rotation.w = 1

        self.p = PoseWithCovarianceStamped()
        self.p.header.frame_id = self.fixed_frame
        self.p.pose.pose.orientation.w = 1

        self.ids = rospy.get_param("ids")
        self.coords = rospy.get_param("positions")
        self.coeffs = rospy.get_param("calibration")
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
            self.subscribers[id] = rospy.Subscriber(
                topic,
                UWBMeas,
                self.range_cb,
                id,
                queue_size=1,
            )
            self.zi[id] = lfilter_zi(self.lp_filter[0], self.lp_filter[1])

        self.last_pos = np.array([np.nan, np.nan])

        self.filter = Kalman(np.array([[0], [0], [0], [0], [0]]), np.eye(5), 0.1)
        self.initialised = False

        self.started = False
        self.pub = rospy.Publisher("/detection_ready", Bool, queue_size=1)
        self.estimate_pub = rospy.Publisher("estimate", PositionEstimate, queue_size=1)
        self.pub2 = rospy.Publisher("pose_cov", PoseWithCovarianceStamped, queue_size=1)
        self.sound_pub = rospy.Publisher("/log_sound", String, queue_size=1)

        rospy.loginfo("Waiting for average value of the range measurements")
        rospy.sleep(5)
        rospy.loginfo("Starting TWR localisation")
        self.tim = rospy.Timer(rospy.Duration(0.1), self.publish_pose)
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        for id in self.ids:
            self.subscribers[id].unregister()
        self.tim.shutdown()
        rospy.sleep(0.5)

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
            self.uwb_stamps[id] = rospy.get_time()

    def intersectionPoint(self, guess):
        x_t = []
        y_t = []
        z_t = []
        d = []
        age = []

        t = rospy.get_time()
        for i in range(len(self.ids)):
            id = self.ids[i]
            x_t += [[self.positions[id][0][0]]]
            y_t += [[self.positions[id][1][0]]]
            z_t += [[self.positions[id][2][0]]]
            d += [[self.ranges_avg[id]]]
            age += [t - self.uwb_stamps[id]]
            if age[-1] > 0.3:
                rospy.logerr(
                    "Measurement from TWR tag %s is more than 0.3 seconds old" % (id)
                )
                id_mod = ""
                for i in range(len(id) - 1):
                    id_mod += id[i] + " "
                id_mod += id[-1]
                s = String(
                    "Warning: Measurement from T W R tag %s is more than 0.3 seconds old"
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
            # f_w = w * f
            f = np.vstack((f,0.3*f_prev))

            return f.flatten().tolist()

        ans = least_squares(eq, guess, loss="soft_l1", verbose=0)

        if ans.success:
            return ans.x
        else:
            return None

    def publish_pose(self, _):
        for id in self.ids:
            if np.isnan(self.ranges[id]):
                rospy.logwarn("missing data")
                return

        if not self.started:
            # send ready signal to convoy node
            self.pub.publish(Bool(True))
            self.started = True

        # find the intersection point
        if np.any(np.isnan(self.last_pos)):
            p = self.positions[self.ids[0]]
            if self.use_3d:
                self.last_pos = np.array(
                    [p[0][0] + self.ranges_avg[self.ids[0]], p[1][0], p[2][0]]
                )
            else:
                self.last_pos = np.array(
                    [p[0][0] + self.ranges_avg[self.ids[0]], p[1][0]]
                )
        x = self.intersectionPoint(self.last_pos)
        if x is None:
            rospy.logwarn("intersection point not found")
            return
        if not self.use_3d:
            x = np.concatenate((x, np.array([self.mount_height])))

        if not self.initialised:
            self.filter.set_initial(np.array([[x[0]], [x[1]], [x[2]]]), np.eye(3))
            self.initialised = True
        else:
            self.filter.correct(np.array([[x[0]], [x[1]], [x[2]]]))
            x_new, cov = self.filter.predict()
            cov_pose = np.zeros((6, 6))
            cov_pose[:3, :3] = cov[:3, :3]
            x = x_new[0:3, :].flatten()

            self.p.header.stamp = rospy.Time.now()
            self.p.pose.pose.position.x = x[0]
            self.p.pose.pose.position.y = x[1]
            self.p.pose.pose.position.z = x[2]

            self.p.pose.covariance = cov_pose.flatten().tolist()

            self.pub2.publish(self.p)
        if self.use_3d:
            self.last_pos = x
        else:
            self.last_pos = x[:2]

        # send estimate
        est = PositionEstimate()
        est.header.frame_id = self.fixed_frame
        est.header.stamp = rospy.Time.now()
        est.position_estimate = Point(x[0], x[1], x[2])
        self.estimate_pub.publish(est)

        # send tf
        self.t.header.stamp = rospy.Time.now()
        self.t.transform.translation.x = x[0]
        self.t.transform.translation.y = x[1]
        self.t.transform.translation.z = x[2]
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
        # weights /= np.linalg.norm(weights)
        return weights


if __name__ == "__main__":
    l = Locator()
    rospy.spin()
