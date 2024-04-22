#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from dwm1001c_ros.msg import UWBMeas
import tf2_ros
import tf2_py as tf2
import ros_numpy
import numpy as np
from numpy.linalg import norm
import sys
import genpy
from follow_me.utils import timer_shutdown, Trajectory, PID, RateLimiter, get_angle


class PurePursuit:
    def __init__(self, value):
        rospy.init_node("pure_pursuit", anonymous=True)
        rospy.sleep(0.2)

        self.base_frame = "body"
        self.odom_frame = "vision"
        self.cmd_topic = "/nav/cmd_vel"
        # self.pos_frame = rospy.get_param("/uwb/twr/human_frame")
        # self.pos_frame = rospy.get_param("/radio/human_frame")
        self.pos_frame = "position_cube"
        self.ready_topic = "/detection_ready"

        self.follow_distance = rospy.get_param("follow_distance")
        self.detect_frequency = rospy.get_param("prediction_frequency")
        self.safe_distance = rospy.get_param("safe_distance")
        self.movement = [0.0, 0.0]
        self.allow_backwards = rospy.get_param("allow_backwards_motion")
        self.backwards = False
        self.control_per = 0.05
        self.switch_count = 0
        self.linear_control = PID(
            0.7, 0.1, 0.5, self.control_per, 0.1, low=0.0, high=0.5
        )
        # self.angular_control = PID(
        #     1.3, 0.04, 1.0, self.control_per, 0.1, low=-0.3, high=0.3
        # )
        # self.angular_control = PID(
        #     0.8, 0.01, 1.0, self.control_per, 0.1, low=-0.2, high=0.2
        # )
        self.angular_control = PID(
            0.6, 0.005, 1.0, self.control_per, 0.1, low=-0.2, high=0.2
        )
        self.linear_limit = RateLimiter(0.9, -0.9, self.control_per)
        self.angular_limit = RateLimiter(1.0, -1.0, self.control_per)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.sleep(0.2)

        self.ids = rospy.get_param("/uwb/twr/ids")
        self.coords = rospy.get_param("/uwb/twr/positions")
        print(self.ids)
        print(self.coords)
        print(len(self.coords))
        self.positions = {}
        self.subscribers = {}
        self.ranges = {}
        for i in range(len(self.ids)):
            id = self.ids[i]
            self.positions[id] = np.array(
                [
                    [self.coords[3 * i]],
                    [self.coords[3 * i + 1]],
                    [self.coords[3 * i + 2]],
                ]
            )
            self.ranges[id] = np.nan
            topic = "/uwb/twr/ID_" + id + "/distances"
            self.subscribers[id] = rospy.Subscriber(
                topic,
                UWBMeas,
                self.d_callback,
                id,
                queue_size=1,
            )

        self.cmd_vel_pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=1)
        self.cmd_timer = rospy.Timer(rospy.Duration(0.02), self.move)
        self.fol_timer = None

        self.sound_pub = rospy.Publisher("/log_sound", String, queue_size=1)

        rospy.on_shutdown(self.shutdown_hook)

    def shutdown_hook(self):
        """invoked on rospy shutdown signal, shutdown all timers and stop the robot

        :return: None
        """
        timer_shutdown(self.fol_timer)
        rospy.sleep(0.2)
        self.movement = [0.0, 0.0]
        rospy.sleep(0.5)
        timer_shutdown(self.cmd_timer)
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

    def d_callback(self, msg, id):
        if len(msg.measurements) != 0:
            self.ranges[id] = msg.measurements[0].dist

    def move(self, _):
        """periodically publish to /nav/cmd_vel

        :param _: from timer (not used)
        :return: None
        """

        tw = Twist()
        tw.linear.y = 0
        tw.linear.z = 0
        tw.angular.x = 0
        tw.angular.y = 0

        all_ready = True
        min_range = np.inf
        for id in self.ids:
            if np.isnan(self.ranges[id]):
                # rospy.logerr("No data from one TWR tag")
                all_ready = False
                break
            min_range = np.min([min_range, self.ranges[id]])
        if all_ready and min_range > self.safe_distance:
            tw.linear.x = np.clip(self.movement[0], -0.5, 1.2)
            tw.angular.z = np.clip(self.movement[1], -0.8, 0.8)
        else:
            # rospy.logwarn_throttle_identical(1.0, "Human is too close, stopping")
            s = String("You are too close, stopping movement")
            self.sound_pub.publish(s)
            self.linear_control.reset()
            self.angular_control.reset()
        self.cmd_vel_pub.publish(tw)

    def follow(self, _):
        t = self.get_transform(self.base_frame, self.pos_frame)
        if t is None:
            return None

        o = np.array([[t[0, 3]], [t[1, 3]]])
        d = norm(o)

        # compute necessary steering inputs
        Va = np.array([[(-1) ** self.backwards], [0], [0]])
        Vb = np.concatenate((o, np.array([[0]])))
        angle = get_angle(Va, Vb, np.array([[0], [0], [1]]))

        # switch between driving forwards and backwards
        if self.allow_backwards and np.abs(angle) > np.deg2rad(150):
            self.switch_count += 1
            if self.switch_count == 5:
                # switch only if the human wants to do it (suppress possible oscillations of the localisation)
                self.switch_count = 0
                rospy.logwarn("SWITCHING DIRECTION OF MOVEMENT")
                s = String("Switching direction of movement")
                self.sound_pub.publish(s)
                self.angular_control.reset()
                self.backwards = not self.backwards
                Va = np.array([[(-1) ** self.backwards], [0], [0]])
                angle = get_angle(Va, Vb, np.array([[0], [0], [1]]))

        # compute error of distance to the leader
        error = d - self.follow_distance

        linear = (-1) ** self.backwards * self.linear_control.control(error)
        angle_err = angle
        angular = self.angular_control.control(angle_err)

        # do not move away from the human to increase the gap
        linear = np.clip(linear, (-1000) * self.backwards, 1000 * (not self.backwards))

        self.movement = [
            self.linear_limit.limit(linear),
            self.angular_limit.limit(angular),
        ]

    def run(self):
        """wait for detection and then start the operation (start timer and spin)"""
        rospy.sleep(0.5)
        try:
            rospy.wait_for_message(self.ready_topic, Bool, timeout=10)
        except:
            rospy.signal_shutdown("Human detection is unavailable")
            s = String("Human detection is unavailable")
            self.sound_pub.publish(s)
        rospy.sleep(0.5)
        s = String("I am ready to follow you")
        self.sound_pub.publish(s)
        self.fol_timer = rospy.Timer(rospy.Duration(self.control_per), self.follow)
        rospy.spin()


if __name__ == "__main__":
    """argv:
    1 - follow distance (meters)
    """
    myargv = rospy.myargv(argv=sys.argv)
    name = ""
    if len(myargv) != 2:
        print("ERROR: Wrong number of arguments")
        quit()
    robot = PurePursuit(float(myargv[1]))
    robot.run()
