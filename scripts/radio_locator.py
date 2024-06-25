#!/usr/bin/env python
# N UWB TWR tags -> 1 anchor + M BT anchors -> 1 BT tag

import rospy

import tf2_ros
from geometry_msgs.msg import (
    TransformStamped,
    Point,
    PoseWithCovarianceStamped,
)
from dwm1001_ros.msg import UWBMeas
from std_msgs.msg import Bool, String
from follow_me.msg import PositionEstimate, HeadingEstimate
from geometry_msgs.msg import Point

import numpy as np
from scipy.optimize import least_squares
from scipy.signal import lfilter, lfilter_zi, butter
from collections import deque
import tf2_ros
import tf2_py as tf2
import ros_numpy
import genpy

R_const = 20
Q_const = 1
CUTOFF = 3.0

np.set_printoptions(precision=3)


def get_angle(Va, Vb, Vn):
    """returns oriented angle of rotation from Va to Vb"""
    # https://stackoverflow.com/a/33920320
    return float(
        np.arctan2(np.matmul(np.cross(Va, Vb, axis=0).T, Vn), np.matmul(Va.T, Vb))
    )


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

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.sleep(0.2)

        self.fixed_frame = rospy.get_param("fixed_frame")
        self.human_frame = rospy.get_param("human_frame")
        self.use_3d = rospy.get_param("use_3d")
        self.mount_height = rospy.get_param("mount_height")
        if not self.use_3d:
            rospy.logwarn("radio localisation is set to operate just in the x-y plane")
        self.br = tf2_ros.TransformBroadcaster()
        self.t = TransformStamped()
        self.t.header.frame_id = self.fixed_frame
        self.t.child_frame_id = self.human_frame
        self.t.transform.rotation.w = 1

        self.p = PoseWithCovarianceStamped()
        self.p.header.frame_id = self.fixed_frame
        self.p.pose.pose.orientation.w = 1

        # TWR
        self.ids = rospy.get_param("/uwb/twr/ids")
        self.coords = rospy.get_param("/uwb/twr/positions")
        self.coeffs = rospy.get_param("/uwb/twr/calibration")
        self.target = rospy.get_param("/uwb/twr/target")
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
            topic = "/uwb/twr/ID_" + id + "/distances"
            self.subscribers[id] = rospy.Subscriber(
                topic,
                UWBMeas,
                self.range_cb,
                id,
                queue_size=1,
            )
            self.zi[id] = lfilter_zi(self.lp_filter[0], self.lp_filter[1])

        # AoA
        self.heading_estimate = None
        self.heading_subs = rospy.Subscriber(
            "/bluetooth/aoa/angle", HeadingEstimate, self.angle_cb
        )

        self.last_pos = np.array([np.nan, np.nan])

        self.filter = Kalman(np.array([[0], [0], [0], [0], [0]]), np.eye(5), 0.1)
        self.initialised = False

        self.started = False
        self.pub = rospy.Publisher("/detection_ready", Bool, queue_size=1)
        self.estimate_pub = rospy.Publisher("estimate", PositionEstimate, queue_size=1)
        # self.pub2 = rospy.Publisher("pose_cov", PoseWithCovarianceStamped, queue_size=1)
        self.sound_pub = rospy.Publisher("/log_sound", String, queue_size=1)

        rospy.loginfo("Waiting for average value of the measurements")
        rospy.sleep(5)
        rospy.loginfo("Starting radio localisation")
        self.tim = rospy.Timer(rospy.Duration(0.1), self.publish_pose)
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        for id in self.ids:
            self.subscribers[id].unregister()
        self.tim.shutdown()
        rospy.sleep(0.5)

    def get_transform(self, tf_from, tf_to, out="matrix", time=None, dur=0.1):
        """returns the latest transformation between the given frames
        the result of multiplying point in frame tf_to by the output matrix is in the frame tf_from

        :param tf_from: find transform from this frame
        :param tf_to: find transform to this frame
        :param out: the return type
                    - 'matrix' - returns numpy array with the tf matrix
                    - 'tf' - returns TransformStamped
        :param time: the desired timestamp of the transform (ROS Time)
        :param dur: the timeout of the lookup (float)
        :return: as selected by out parameter or None in case of tf2 exception
                    - only ConnectivityException is logged
        """
        if time is None:
            tf_time = rospy.Time(0)
        else:
            if not isinstance(time, rospy.Time) and not isinstance(time, genpy.Time):
                raise TypeError("parameter time has to be ROS Time")
            tf_time = time

        try:
            t = self.tf_buffer.lookup_transform(
                tf_from, tf_to, tf_time, rospy.Duration(dur)
            )
        except (tf2.LookupException, tf2.ExtrapolationException):
            return None
        except tf2.ConnectivityException as ex:
            rospy.logerr(ex)
            return None

        # return the selected type
        if out == "matrix":
            return ros_numpy.numpify(t.transform)
        elif out == "tf":
            return t
        else:
            raise ValueError("argument out should be 'matrix' or 'tf'")

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

    def angle_cb(self, msg):
        self.heading_estimate = msg

    def intersectionPoint(self, guess, init, p_init):
        if self.heading_estimate is None:
            rospy.logwarn("No estimate available")
            return None
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
        angles = [self.heading_estimate.azimuth, self.heading_estimate.elevation]
        angles_frame = self.heading_estimate.header.frame_id
        tf = self.get_transform(angles_frame, self.fixed_frame)
        if tf is None:
            rospy.logfatal(
                "No transform between %s and %s" % (self.fixed_frame, angles_frame)
            )
            return None

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
            # f_w = w * f  # weighting to suppress incorrect measurement

            # AOA
            n = np.array([[np.cos(angles[0])], [np.sin(angles[0])], [0]])
            p = np.matmul(tf, np.array([[x], [y], [0], [1]]))[:3, :]
            p[2, :] = 0.0
            angle = get_angle(n, p, np.array([[0.0], [0.0], [1.0]]))
            f = np.vstack((f, 3.0 * np.abs(float(angle))))

            # no large jumps between iterations
            # if self.use_3d:
            #     f_prev = (x - guess[0]) ** 2 + (y - guess[1]) ** 2 + (z - guess[2]) ** 2
            # else:
            #     f_prev = (x - guess[0]) ** 2 + (y - guess[1]) ** 2
            # f = np.vstack((f,0.3*f_prev))

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

    def publish_pose(self, _):
        for id in self.ids:
            if np.isnan(self.ranges[id]):
                rospy.logwarn("missing data")
                return

        # find the intersection point
        init = False
        p_init = []
        if np.any(np.isnan(self.last_pos)):
            p = self.positions[self.ids[0]]
            if self.use_3d:
                p_init += np.array(
                    [p[0][0] + self.ranges_avg[self.ids[0]], p[1][0], p[2][0]]
                )
                p_init += np.array(
                    [p[0][0], p[1][0] + self.ranges_avg[self.ids[0]], p[2][0]]
                )
                p_init += np.array(
                    [p[0][0] - self.ranges_avg[self.ids[0]], p[1][0], p[2][0]]
                )
                p_init += np.array(
                    [p[0][0], p[1][0] - self.ranges_avg[self.ids[0]], p[2][0]]
                )
            else:
                p_init += np.array([p[0][0] + self.ranges_avg[self.ids[0]], p[1][0]])
                p_init += np.array([p[0][0], p[1][0]] + self.ranges_avg[self.ids[0]])
                p_init += np.array([p[0][0] - self.ranges_avg[self.ids[0]], p[1][0]])
                p_init += np.array([p[0][0], p[1][0]] - self.ranges_avg[self.ids[0]])
            init = True
        x = self.intersectionPoint(self.last_pos, init, p_init)
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

            # self.pub2.publish(self.p)
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

        # send ready signal
        if not self.started:
            self.pub.publish(Bool(True))
            self.started = True

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
