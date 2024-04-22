#!/usr/bin/env python

import rospy
import numpy as np
from collections import deque
from numpy.linalg import norm
import math


def wait_until(time):
    while not rospy.is_shutdown():
        t = rospy.get_time()
        d = time - t
        if d < 0:
            return
        rospy.sleep(d / 2)
        if d <= 0.02:
            return


def timer_shutdown(timer):
    if timer is not None:
        timer.shutdown()


def test_topic(name):
    t = rospy.get_published_topics()
    found = False
    for i in range(len(t)):
        if t[i][0] == name:
            found = True
            break
    return found


class RateLimiter:
    def __init__(self, positive, negative, period):
        self.pos = positive
        self.neg = negative
        self.per = period
        self.last_value = None
        self.last_time = None

    def limit(self, value):
        output = 0
        if self.last_value is None:
            output = value
            self.last_value = output
            self.last_time = rospy.get_time()
        else:
            diff = value - self.last_value
            per = None
            if self.per > 0:
                per = self.per
            else:
                now = rospy.get_time()
                per = now - self.last_time
            output = self.last_value + np.clip(diff, self.neg * per, self.pos * per)
            self.last_time = rospy.get_time()
            self.last_value = output
        return output


class TrajectoryPoint:
    def __init__(self, point, stamp):
        """trajectory point with timestamp

        :param point: the trajectory point as a numpy array
        :param stamp: the timestamp as a ROS Time
        """

        self.point = point
        self.stamp = stamp


class Trajectory(deque):
    def __init__(self, max_heading_change):
        """trajectory for convoying, uses deque to store points (type TrajectoryPoint)"""

        deque.__init__(self)
        self.max_diff = max_heading_change

    def safe_pop(self):
        """return trajectory point or None if queue is empty

        :return: TrajectoryPoint or None"""

        if len(self) != 0:
            return self.popleft()
        else:
            return None

    def time_pop(self, now):
        """return the first point of the trajectory in the future or the last point of the trajectory

        :param now: current ROS Time
        :return: first TrajectoryPoint with future stamp or the last point in the queue or None (empty queue)
        """

        p = None
        while not rospy.is_shutdown():
            tmp = self.safe_pop()
            if p is None and tmp is None:
                # queue is empty -> return None
                return None
            elif tmp is None:
                # queue is empty now, but there was at least one point in it -> return the last point
                return p
            elif tmp.stamp - now > 0:
                # stamp is in the future -> return the point
                return tmp
            p = tmp

    def append_point(self, point, stamp):
        """append TrajectoryPoint to the queue

        :param point: the trajectory point
        :param stamp: the timestamp (expected time of reaching the point)"""
        if len(self) == 0 or (
            len(self) != 0 and norm(point[0:2] - self[-1].point[0:2]) >= 0.1
        ):
            # don't add points, that are too close to each other (leader is probably standing)
            if len(self) >= 2:
                # smooth the trajectory by not allowing big changes of heading (vector between two points)
                old_dir = self[-1].point[0:2]-self[-2].point[0:2]
                old_dir3 = self[-1].point - self[-2].point
                old_dir3_norm = old_dir3/norm(old_dir3)

                new_dir = point[0:2]-self[-1].point[0:2]
                n = norm(new_dir)

                Va = np.concatenate((old_dir, np.array([[0]])))
                Vb = np.concatenate((new_dir, np.array([[0]])))
                angle = get_angle(Va, Vb, np.array([[0],[0],[1]]))
                if abs(angle) > self.max_diff:
                    new_angle = np.sign(angle)*self.max_diff
                    if abs(angle) > math.pi/2:
                        #return
                        p = TrajectoryPoint(point,stamp)
                        self.append(p)
                    # clip the heading change
                    c = np.cos(new_angle)
                    s = np.sin(new_angle)
                    mat = np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
                    direction = 0.7 * n * np.matmul(mat, old_dir3_norm)
                    # apply the modified direction
                    #point = self[-1].point + direction
            p = TrajectoryPoint(point, stamp)
            self.append(p)

    def distance_along(self, start=None):
        """return the distance along the trajectory from the first to the last point (position of the leader)
        in the xy plane

        :param start: the starting point of the computation
                        - np.array - start is the current position of the follower
                        - None - returns the length of the trajectory in the queue
        :return: distance along the trajectory (float)"""

        sum = 0
        if len(self) != 0:
            if start is not None:
                sum += norm(self[0].point[0:2] - start)
            for i in range(1, len(self)):
                sum += norm(self[i].point[0:2] - self[i - 1].point[0:2])
        return sum


class PID:
    def __init__(self, k_p, k_i, k_d, t_s, n, anti_windup=True, low=-1.0, high=1.0):
        """PID controller with anti-windup (clamping) and derivative filtering

        :param float k_p: proportional (float)
        :param float k_i: integral (float)
        :param float k_d: derivative (float)
        :param float t_s: sampling time (float)
        :param float n: derivative filtering coefficient
        :param bool anti_windup: turn anti-windup on/off (default True)
        :param float low: lower limit for clamping (default -1.)
        :param float high: upper limit for clamping (default 1.)
        """
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.t_s = t_s
        self.n = n

        self.last_err = None
        self.filter = 0
        self.i_term = 0

        # set anti-windup (clamping)
        self.anti_windup = anti_windup
        self.low_lim = low
        self.high_lim = high

    def control(self, err):
        """compute the output with PID with the given parameters

        :param float err: the error of the input with respect to the setpoint

        :return: the control variable
        """
        # compute integral term
        self.i_term += self.k_i * err * self.t_s

        # compute derivative term
        diff = self.n * (err * self.k_d - self.filter)
        self.filter = self.filter + self.t_s * diff
        # diff = 0
        # if self.last_err is not None:
        #     diff = (err-self.last_err)/self.t_s

        if self.anti_windup:
            # perform clamping if enabled
            self.i_term = np.clip(self.i_term, self.low_lim, self.high_lim)

        out = self.k_p * err + self.i_term + self.k_d * diff
        self.last_err = err
        return out
    
    def reset(self):
        self.i_term = 0.
        self.filter = 0.
        self.last_err = None


def get_angle(Va, Vb, Vn):
    """returns oriented angle of rotation from Va to Vb"""
    # https://stackoverflow.com/a/33920320
    return float(
        np.arctan2(np.matmul(np.cross(Va, Vb, axis=0).T, Vn), np.matmul(Va.T, Vb))
    )
