#!/usr/bin/env python

import rospy

from xplraoa_ros.msg import Angles
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, TransformStamped, Quaternion, Vector3
from follow_me.msg import HeadingEstimate

import numpy as np
import tf2_ros
import tf2_py as tf2
import ros_numpy
import genpy
import threading
from collections import deque
from copy import deepcopy

ANGLE_LIMIT = np.deg2rad(70.0)
CUTOFF = 1.0
Q_LEN = 5
BEST_LEN = 3
R = 10
Q = 1


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
        self.Q = Q * np.eye(3)
        self.R = R * np.eye(3)
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


def get_angle(Va, Vb, Vn):
    """returns oriented angle of rotation from Va to Vb"""
    # https://stackoverflow.com/a/33920320
    return float(
        np.arctan2(np.matmul(np.cross(Va, Vb, axis=0).T, Vn), np.matmul(Va.T, Vb))
    )


class Locator:
    def __init__(self):
        rospy.init_node("aoa_locator")
        t1 = rospy.get_time()
        self.msg_lock = threading.Lock()

        # tf2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.br_static = tf2_ros.StaticTransformBroadcaster()
        rospy.sleep(0.2)

        # read AoA params
        self.mode = rospy.get_param("mode")
        self.estimate_frame = rospy.get_param("estimate_frame")
        self.fixed_frame = rospy.get_param("fixed_frame")
        self.ids = rospy.get_param("ids")
        self.coords = rospy.get_param("positions")
        frames = rospy.get_param("frames")
        self.poses = {}
        self.subscribers = {}
        self.markers = {}
        self.frames = {}
        self.angles = {}
        self.rssi = {}
        tfs = []
        # prepare rviz markers and measurement storage for each AoA anchor
        for i in range(len(self.ids)):
            id = self.ids[i]

            # static tf between the fixed frame and antenna board center as specified by rosparam
            p = Vector3(
                self.coords[7 * i], self.coords[7 * i + 1], self.coords[7 * i + 2]
            )
            q = Quaternion(
                self.coords[7 * i + 3],
                self.coords[7 * i + 4],
                self.coords[7 * i + 5],
                self.coords[7 * i + 6],
            )
            self.poses[id] = (p, q)
            self.frames[id] = frames[i]
            tf = TransformStamped()
            tf.header.frame_id = self.fixed_frame
            tf.header.stamp = rospy.Time.now()
            tf.child_frame_id = frames[i]
            tf.transform.translation = p
            tf.transform.rotation = q
            tfs += [tf]

            # subscriber and measurement queue
            topic = id + "/angles_avg"
            self.subscribers[id] = rospy.Subscriber(
                topic,
                Angles,
                self.aoa_cb,
                id,
                queue_size=1,
            )
            self.angles[id] = deque()
            self.rssi[id] = deque()

            # rviz visualization of measured angles
            marker = Marker()
            marker.header.frame_id = frames[i]
            marker.type = marker.ARROW
            marker.action = marker.ADD
            marker.scale.x = 0.05
            marker.scale.y = 0.1
            marker.scale.z = 0.15
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            self.markers[id] = marker
        self.br_static.sendTransform(tfs)

        # angle estimation
        self.best_antenna = deque()
        self.angle_pub = rospy.Publisher("angle", HeadingEstimate, queue_size=1)
        self.marker_pub = rospy.Publisher("heading", Marker, queue_size=1)
        self.markers_pub = rospy.Publisher("measured_angles", MarkerArray, queue_size=1)

        # rviz visualization of the final angles
        self.marker = Marker()
        self.marker.header.frame_id = None
        self.marker.type = self.marker.ARROW
        self.marker.action = self.marker.ADD
        self.marker.scale.x = 0.1
        self.marker.scale.y = 0.2
        self.marker.scale.z = 0.3
        self.marker.color.a = 1.0
        self.marker.color.r = 0.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0
        self.marker.pose.position.x = 0.0
        self.marker.pose.position.y = 0.0
        self.marker.pose.position.z = 0.0
        self.marker.pose.orientation.w = 1.0

        # Kalman
        self.filter = Kalman(np.array([[0], [0], [0]]), np.eye(3), 0.1)
        self.initialised = False

        # start the desired localisation method
        if self.mode == "RSSI":
            self.tim = rospy.Timer(rospy.Duration(0.05), self.estimate_angle_rssi)
        elif self.mode == "external":
            self.tim = rospy.Timer(rospy.Duration(0.05), self.estimate_angle_external)
        else:
            rospy.logerr("Invalid mode specified, using RSSI")
            self.tim = rospy.Timer(rospy.Duration(0.05), self.estimate_angle_rssi)

        rospy.on_shutdown(self.shutdown)
        t2 = rospy.get_time()
        # print("startup: %f" % (t2-t1))

    def shutdown(self):
        self.tim.shutdown()
        rospy.sleep(0.2)

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

    def aoa_cb(self, msg, id):
        """store the incomming measurement in the queue, keep the queue at the desired size"""
        t1 = rospy.get_time()
        self.msg_lock.acquire()
        self.angles[id].append(np.array([[msg.azimuth], [msg.elevation]]))
        self.rssi[id].append(msg.RSSI)
        while len(self.angles[id]) > Q_LEN:
            self.angles[id].popleft()
            self.rssi[id].popleft()
        self.msg_lock.release()
        t2 = rospy.get_time()
        # print("angle cb: %f" % (t2-t1))

    def estimate_angle_rssi(self, _):
        """estimate angle by selecting the antenna with the best RSSI, keep the choice consistent
        (do not allow the estimate to jump back and forth between two antennas)"""
        t1 = rospy.get_time()
        self.msg_lock.acquire()
        rssi_cp = deepcopy(self.rssi)
        angles_cp = deepcopy(self.angles)
        self.msg_lock.release()

        # find antenna with highest RSSI (mean of last few measurements)
        max_rssi = -np.inf
        selected_id = None
        target_angles = None
        markers = MarkerArray()
        for id in self.ids:
            if len(angles_cp[id]) != 0 and len(rssi_cp[id]) != 0:
                arr = np.array(rssi_cp[id])
                mean = np.mean(arr)
                std = np.std(arr)
                mean_mod = mean - 2 * std
                az = angles_cp[id][-1][0]
                el = angles_cp[id][-1][1]
                scale = 5 * np.log10((mean + 100) / 10)
                self.markers[id].header.stamp = rospy.Time.now()
                self.markers[id].points = [
                    Point(0, 0, 0),
                    Point(
                        scale * np.cos(el) * np.cos(az),
                        scale * np.cos(el) * np.sin(az),
                        scale * np.sin(el),
                    ),
                ]
                markers.markers.append(self.markers[id])

                if mean_mod > max_rssi:
                    selected_id = id
                    target_angles = angles_cp[id][-1]
                    max_rssi = mean_mod
        id = 0
        for m in markers.markers:
            m.id = id
            id += 1
        self.markers_pub.publish(markers)
        if selected_id is None:
            # no measurement available
            return
        # add the current best antenna to the queue
        self.best_antenna.append(selected_id)
        while len(self.best_antenna) > BEST_LEN:
            self.best_antenna.popleft()
        if len(self.best_antenna) == BEST_LEN:
            # check that the selected antenna is consistent with history
            values, counts = np.unique(self.best_antenna, return_counts=True)
            ind = np.argmax(counts)
            most_frequent = values[ind]
            selected_id = most_frequent
            target_angles = angles_cp[most_frequent][-1]

        # filter using linear Kalman
        az = float(target_angles[0])
        el = float(target_angles[1])
        p = np.array(
            [[np.cos(el) * np.cos(az)], [np.cos(el) * np.sin(az)], [np.sin(el)]]
        )
        t = self.get_transform(self.fixed_frame, self.frames[selected_id])
        if t is None:
            return
        p = np.matmul(t, np.vstack((p, np.array([[1]]))))[:3, :]
        if not self.initialised:
            self.filter.set_initial(p, np.eye(3))
            self.initialised = True
        else:
            self.filter.correct(p)
            x_new, cov = self.filter.predict()
            x_new /= np.linalg.norm(x_new)

            Va = np.array([[1], [0], [0]])
            Vn1 = np.array([[0], [0], [1]])
            az_f = get_angle(Va, x_new, Vn1)

            c = np.cos(-az_f)
            s = np.sin(-az_f)
            t2 = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            x2 = np.matmul(t2, x_new)
            Vn2 = np.array([[0], [1], [0]])
            el_f = get_angle(x2, Va, Vn2)

            # publish the result as an estimate message (angles and reference tf frame)
            estimate = HeadingEstimate()
            estimate.header.frame_id = self.fixed_frame
            estimate.header.stamp = rospy.Time.now()
            estimate.azimuth = az_f
            estimate.elevation = el_f
            self.angle_pub.publish(estimate)

            # publish rviz arrow for the final angles
            self.marker.header.stamp = rospy.Time.now()
            self.marker.header.frame_id = self.fixed_frame
            self.marker.points = [
                Point(0, 0, 0),
                Point(
                    3.0 * np.cos(el_f) * np.cos(az_f),
                    3.0 * np.cos(el_f) * np.sin(az_f),
                    3.0 * np.sin(el_f),
                ),
            ]
            self.marker_pub.publish(self.marker)
        t2 = rospy.get_time()
        # print("estimate: %f" % (t2-t1))

    def estimate_angle_external(self, _):
        """use external estimate of transmitters position (i.e. uwb) to select antenna that has the transmitter in range"""
        t1 = rospy.get_time()
        # compute heading towards the estimated position for each antenna board
        in_range = []
        Va = np.array([[1], [0], [0]])
        Vn = np.array([[0], [0], [1]])
        markers = MarkerArray()
        self.msg_lock.acquire()
        for id in self.ids:
            t = self.get_transform(self.frames[id], self.estimate_frame)
            if t is None:
                rospy.logwarn("No tf")
                self.msg_lock.release()
                return
            Vb = t[0:3, 3:4]
            angle = float(get_angle(Va, Vb, Vn))
            if np.abs(angle) <= ANGLE_LIMIT:
                in_range += [(id, angle, self.rssi[id])]
            if len(self.angles[id]) != 0:
                az = self.angles[id][-1][0]
                el = self.angles[id][-1][1]
                self.markers[id].header.stamp = rospy.Time.now()
                self.markers[id].points = [
                    Point(0, 0, 0),
                    Point(
                        0.5 * np.cos(el) * np.cos(az),
                        0.5 * np.cos(el) * np.sin(az),
                        0.5 * np.sin(el),
                    ),
                ]
                markers.markers.append(self.markers[id])
            else:
                rospy.logwarn("Queue is empty")
                return
        best = -np.inf
        selected_id = None
        target_angles = None
        for antenna in in_range:
            # select the one that has the highest RSSI
            # if np.abs(antenna[1]) < best:
            if antenna[2] > best:
                target_angles = self.angles[antenna[0]][-1]
                selected_id = antenna[0]
                best = antenna[2]
        self.msg_lock.release()
        if selected_id is None:
            return
        id = 0
        for m in markers.markers:
            m.id = id
            id += 1
        self.markers_pub.publish(markers)
        if len(in_range) == 0:
            rospy.logerr("No AoA antenna in range")
            return

        # publish the result as an estimate (angles and reference tf frame)
        estimate = HeadingEstimate()
        estimate.header.frame_id = self.frames[selected_id]
        estimate.header.stamp = rospy.Time.now()
        estimate.azimuth = target_angles[0]
        estimate.elevation = target_angles[1]
        self.angle_pub.publish(estimate)

        # publish marker for the final angles
        self.marker.header.stamp = rospy.Time.now()
        self.marker.header.frame_id = self.frames[selected_id]
        az = target_angles[0]
        el = target_angles[1]
        self.marker.points = [
            Point(0, 0, 0),
            Point(
                3.0 * np.cos(el) * np.cos(az),
                3.0 * np.cos(el) * np.sin(az),
                3.0 * np.sin(el),
            ),
        ]
        self.marker_pub.publish(self.marker)
        t2 = rospy.get_time()
        # print("estimate2: %f" % (t2-t1))


if __name__ == "__main__":
    l = Locator()
    rospy.spin()
