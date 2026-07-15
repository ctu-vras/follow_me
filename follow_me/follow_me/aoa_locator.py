#!/usr/bin/env python3

import threading
from collections import deque
from copy import deepcopy

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener
from xplraoa_ros_interfaces.msg import Angles
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, TransformStamped, Quaternion, Vector3
from follow_me_interfaces.msg import HeadingEstimate

from follow_me.utils import (
    Kalman,
    attach_kalman_param_callback,
    declare_kalman_parameters,
    get_angle,
    get_transform,
)

ANGLE_LIMIT = np.deg2rad(70.0)
CUTOFF = 1.0
Q_LEN = 5
BEST_LEN = 3


class Locator(Node):
    def __init__(self):
        super().__init__("aoa_locator")
        self.msg_lock = threading.Lock()

        # tf2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.br_static = StaticTransformBroadcaster(self)

        # read AoA params
        self.declare_parameter("mode", "RSSI")
        self.declare_parameter("estimate_frame", Parameter.Type.STRING)
        self.declare_parameter("fixed_frame", Parameter.Type.STRING)
        self.declare_parameter("ids", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("positions", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("frames", Parameter.Type.STRING_ARRAY)

        self.mode = self.get_parameter("mode").value
        self.estimate_frame = self.get_parameter("estimate_frame").value
        self.fixed_frame = self.get_parameter("fixed_frame").value
        self.ids = self.get_parameter("ids").value
        self.coords = self.get_parameter("positions").value
        frames = self.get_parameter("frames").value
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

            # static tf between the fixed frame and antenna board center as specified by param
            p = Vector3(
                x=float(self.coords[7 * i]),
                y=float(self.coords[7 * i + 1]),
                z=float(self.coords[7 * i + 2]),
            )
            q = Quaternion(
                x=float(self.coords[7 * i + 3]),
                y=float(self.coords[7 * i + 4]),
                z=float(self.coords[7 * i + 5]),
                w=float(self.coords[7 * i + 6]),
            )
            self.poses[id] = (p, q)
            self.frames[id] = frames[i]
            tf = TransformStamped()
            tf.header.frame_id = self.fixed_frame
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.child_frame_id = frames[i]
            tf.transform.translation = p
            tf.transform.rotation = q
            tfs += [tf]

            # subscriber and measurement queue
            topic = id + "/angles_avg"
            self.subscribers[id] = self.create_subscription(
                Angles, topic, lambda msg, id=id: self.aoa_cb(msg, id), 1
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
        self.angle_pub = self.create_publisher(HeadingEstimate, "angle", 1)
        self.marker_pub = self.create_publisher(Marker, "heading", 1)
        self.markers_pub = self.create_publisher(MarkerArray, "measured_angles", 1)

        # rviz visualization of the final angles
        self.marker = Marker()
        self.marker.header.frame_id = ""
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
        # r is scaled to the state itself (a unit direction vector, not meters):
        # the previous r=10 was ~30x the state's own magnitude, which (combined
        # with the predict/correct ordering bug) made the innovation gate below
        # essentially never fire - any jump looked statistically unremarkable
        # next to noise that huge. gate_threshold/max_inflation control how
        # aggressively real jumps (e.g. the robot turning suddenly) get absorbed
        # in ~1-2 cycles instead of smoothed away over seconds; q is the nominal
        # per-second process noise for calm conditions.
        kalman_params = declare_kalman_parameters(self, q=1.0, r=0.05)
        self.filter = Kalman(np.array([[0.0], [0.0], [0.0]]), np.eye(3), **kalman_params)
        attach_kalman_param_callback(self, self.filter)
        self.filter_last_time = None
        self.initialised = False

        # start the desired localisation method
        if self.mode == "RSSI":
            self.tim = self.create_timer(0.05, self.estimate_angle_rssi)
        elif self.mode == "external":
            self.tim = self.create_timer(0.05, self.estimate_angle_external)
        else:
            self.get_logger().error("Invalid mode specified, using RSSI")
            self.tim = self.create_timer(0.05, self.estimate_angle_rssi)

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def aoa_cb(self, msg, id):
        """store the incomming measurement in the queue, keep the queue at the desired size"""
        with self.msg_lock:
            self.angles[id].append(np.array([[msg.azimuth], [msg.elevation]]))
            self.rssi[id].append(msg.rssi)
            while len(self.angles[id]) > Q_LEN:
                self.angles[id].popleft()
                self.rssi[id].popleft()

    def estimate_angle_rssi(self):
        """estimate angle by selecting the antenna with the best RSSI, keep the choice consistent
        (do not allow the estimate to jump back and forth between two antennas)"""
        with self.msg_lock:
            rssi_cp = deepcopy(self.rssi)
            angles_cp = deepcopy(self.angles)

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
                self.markers[id].header.stamp = self.get_clock().now().to_msg()
                self.markers[id].points = [
                    Point(x=0.0, y=0.0, z=0.0),
                    Point(
                        x=float(scale * np.cos(el) * np.cos(az)),
                        y=float(scale * np.cos(el) * np.sin(az)),
                        z=float(scale * np.sin(el)),
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
        t = get_transform(self, self.tf_buffer, self.fixed_frame, self.frames[selected_id])
        if t is None:
            return
        antenna_origin = t[:3, 3:4]
        p = np.matmul(t, np.vstack((p, np.array([[1]]))))[:3, :]
        now = self.now_sec()
        if not self.initialised:
            self.filter.set_initial(p, np.eye(3))
            self.initialised = True
            self.filter_last_time = now
        else:
            dt = now - self.filter_last_time
            self.filter_last_time = now
            x_new, cov = self.filter.step(p, dt)
            direction = x_new - antenna_origin
            direction /= np.linalg.norm(direction)

            Va = np.array([[1], [0], [0]])
            Vn1 = np.array([[0], [0], [1]])
            az_f = get_angle(Va, direction, Vn1)

            cc = np.cos(-az_f)
            s = np.sin(-az_f)
            t2 = np.array([[cc, -s, 0], [s, cc, 0], [0, 0, 1]])
            x2 = np.matmul(t2, direction)
            Vn2 = np.array([[0], [1], [0]])
            el_f = get_angle(x2, Va, Vn2)

            # publish the result as an estimate message (angles and reference tf frame)
            estimate = HeadingEstimate()
            estimate.header.frame_id = self.fixed_frame
            estimate.header.stamp = self.get_clock().now().to_msg()
            estimate.azimuth = float(az_f)
            estimate.elevation = float(el_f)
            self.angle_pub.publish(estimate)

            # publish rviz arrow for the final angles
            self.marker.header.stamp = self.get_clock().now().to_msg()
            self.marker.header.frame_id = self.fixed_frame
            self.marker.points = [
                Point(x=0.0, y=0.0, z=0.0),
                Point(
                    x=float(3.0 * np.cos(el_f) * np.cos(az_f)),
                    y=float(3.0 * np.cos(el_f) * np.sin(az_f)),
                    z=float(3.0 * np.sin(el_f)),
                ),
            ]
            self.marker_pub.publish(self.marker)

    def estimate_angle_external(self):
        """use external estimate of transmitters position (i.e. uwb) to select antenna that has the transmitter in range"""
        # compute heading towards the estimated position for each antenna board
        in_range = []
        Va = np.array([[1], [0], [0]])
        Vn = np.array([[0], [0], [1]])
        markers = MarkerArray()
        with self.msg_lock:
            for id in self.ids:
                t = get_transform(
                    self, self.tf_buffer, self.frames[id], self.estimate_frame
                )
                if t is None:
                    self.get_logger().warn("No tf")
                    return
                Vb = t[0:3, 3:4]
                angle = float(get_angle(Va, Vb, Vn))
                if np.abs(angle) <= ANGLE_LIMIT:
                    in_range += [(id, angle, self.rssi[id])]
                if len(self.angles[id]) != 0:
                    az = self.angles[id][-1][0]
                    el = self.angles[id][-1][1]
                    self.markers[id].header.stamp = self.get_clock().now().to_msg()
                    self.markers[id].points = [
                        Point(x=0.0, y=0.0, z=0.0),
                        Point(
                            x=float(0.5 * np.cos(el) * np.cos(az)),
                            y=float(0.5 * np.cos(el) * np.sin(az)),
                            z=float(0.5 * np.sin(el)),
                        ),
                    ]
                    markers.markers.append(self.markers[id])
                else:
                    self.get_logger().warn("Queue is empty")
                    return
            best = -np.inf
            selected_id = None
            target_angles = None
            for antenna in in_range:
                # select the one that has the highest RSSI
                if antenna[2] > best:
                    target_angles = self.angles[antenna[0]][-1]
                    selected_id = antenna[0]
                    best = antenna[2]
        if selected_id is None:
            return
        id = 0
        for m in markers.markers:
            m.id = id
            id += 1
        self.markers_pub.publish(markers)
        if len(in_range) == 0:
            self.get_logger().error("No AoA antenna in range")
            return

        # publish the result as an estimate (angles and reference tf frame)
        estimate = HeadingEstimate()
        estimate.header.frame_id = self.frames[selected_id]
        estimate.header.stamp = self.get_clock().now().to_msg()
        estimate.azimuth = float(target_angles[0])
        estimate.elevation = float(target_angles[1])
        self.angle_pub.publish(estimate)

        # publish marker for the final angles
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.marker.header.frame_id = self.frames[selected_id]
        az = target_angles[0]
        el = target_angles[1]
        self.marker.points = [
            Point(x=0.0, y=0.0, z=0.0),
            Point(
                x=float(3.0 * np.cos(el) * np.cos(az)),
                y=float(3.0 * np.cos(el) * np.sin(az)),
                z=float(3.0 * np.sin(el)),
            ),
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
