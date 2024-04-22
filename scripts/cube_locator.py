#!/usr/bin/env python
# 1 UWB TWR tags -> 1 anchor + 4 BT anchors -> 1 BT tag

import rospy

import tf2_ros
from geometry_msgs.msg import (
    TransformStamped,
    Point,
    Vector3,
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
import time

R_const = 2
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


class Locator:
    def __init__(self):
        rospy.init_node("twr_locator")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.sleep(0.2)

        self.fixed_frame = "aoa_top_mount"
        self.human_frame = "position_cube"
        self.br = tf2_ros.TransformBroadcaster()
        self.t = TransformStamped()
        self.t.header.frame_id = self.fixed_frame
        self.t.child_frame_id = self.human_frame
        self.t.transform.rotation.w = 1

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
        self.twr_subscriber = rospy.Subscriber(
            topic,
            UWBMeas,
            self.range_cb,
            id,
            queue_size=1,
        )

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
        self.sound_pub = rospy.Publisher("/log_sound", String, queue_size=1)

        rospy.loginfo("Waiting for average value of the measurements")
        rospy.sleep(5)
        rospy.loginfo("Starting radio localisation")
        self.tim = rospy.Timer(rospy.Duration(0.1), self.publish_pose)
        rospy.on_shutdown(self.shutdown)

    def shutdown(self):
        self.twr_subscriber.unregister()
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
        if len(msg.measurements) != 0:
            d = msg.measurements[0].dist
            self.range = d
            # p = self.calibration[id]
            # d_cal = p[0] * d**3 + p[1] * d**2 + p[2] * d + p[3]
            d_cal = d
            d_filt, zi = lfilter(
                self.lp_filter[0], self.lp_filter[1], np.array([d_cal]), zi=self.zi
            )
            self.zi = zi
            self.range = d
            self.range_avg = float(d_filt)
            self.uwb_stamp = rospy.get_time()

    def angle_cb(self, msg):
        self.heading_estimate = msg

    def intersectionPoint(self):
        if self.heading_estimate is None or np.isnan(self.range):
            rospy.logwarn("No data available yet")
            return None

        (az, el) = [self.heading_estimate.azimuth, self.heading_estimate.elevation]
        angles_frame = self.heading_estimate.header.frame_id
        tf = self.get_transform(self.fixed_frame, angles_frame)
        if tf is None:
            rospy.logfatal(
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

    def publish_pose(self, _):
        # find the intersection point
        x = self.intersectionPoint()
        if x is None:
            rospy.logwarn("intersection point not found")
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


if __name__ == "__main__":
    l = Locator()
    rospy.spin()
