import math
from collections import deque

import numpy as np
import rclpy
import rclpy.time
import ros2_numpy
from numpy.linalg import norm
from rcl_interfaces.msg import SetParametersResult
from rclpy.duration import Duration
from tf2_ros import (
    ConnectivityException,
    ExtrapolationException,
    LookupException,
)


def _now_sec(clock):
    """current time of an rclpy Clock in floating point seconds"""
    return clock.now().nanoseconds * 1e-9


def wait_until(node, time):
    """block until the given time (float seconds) is reached, spinning the node"""
    while rclpy.ok():
        t = _now_sec(node.get_clock())
        d = time - t
        if d < 0:
            return
        rclpy.spin_once(node, timeout_sec=d / 2)
        if d <= 0.02:
            return


def timer_shutdown(timer):
    if timer is not None:
        timer.cancel()


def test_topic(node, name):
    t = node.get_topic_names_and_types()
    found = False
    for i in range(len(t)):
        if t[i][0] == name:
            found = True
            break
    return found


class RateLimiter:
    def __init__(self, positive, negative, period, clock=None):
        self.pos = positive
        self.neg = negative
        self.per = period
        self.clock = clock
        self.last_value = None
        self.last_time = None

    def limit(self, value):
        output = 0
        if self.last_value is None:
            output = value
            self.last_value = output
            self.last_time = _now_sec(self.clock) if self.clock is not None else None
        else:
            diff = value - self.last_value
            per = None
            if self.per > 0:
                per = self.per
            else:
                now = _now_sec(self.clock)
                per = now - self.last_time
            output = self.last_value + np.clip(diff, self.neg * per, self.pos * per)
            if self.clock is not None:
                self.last_time = _now_sec(self.clock)
            self.last_value = output
        return output


class TrajectoryPoint:
    def __init__(self, point, stamp):
        """trajectory point with timestamp

        :param point: the trajectory point as a numpy array
        :param stamp: the timestamp in floating point seconds
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

        :param now: current time in floating point seconds
        :return: first TrajectoryPoint with future stamp or the last point in the queue or None (empty queue)
        """

        p = None
        while True:
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
                old_dir = self[-1].point[0:2] - self[-2].point[0:2]
                old_dir3 = self[-1].point - self[-2].point
                old_dir3_norm = old_dir3 / norm(old_dir3)

                new_dir = point[0:2] - self[-1].point[0:2]
                n = norm(new_dir)

                Va = np.concatenate((old_dir, np.array([[0]])))
                Vb = np.concatenate((new_dir, np.array([[0]])))
                angle = get_angle(Va, Vb, np.array([[0], [0], [1]]))
                if abs(angle) > self.max_diff:
                    new_angle = np.sign(angle) * self.max_diff
                    if abs(angle) > math.pi / 2:
                        # return
                        p = TrajectoryPoint(point, stamp)
                        self.append(p)
                    # clip the heading change
                    c = np.cos(new_angle)
                    s = np.sin(new_angle)
                    mat = np.array(
                        [[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
                    )
                    direction = 0.7 * n * np.matmul(mat, old_dir3_norm)
                    # apply the modified direction
                    # point = self[-1].point + direction
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

        if self.anti_windup:
            # perform clamping if enabled
            self.i_term = np.clip(self.i_term, self.low_lim, self.high_lim)

        out = self.k_p * err + self.i_term + self.k_d * diff
        self.last_err = err
        return out

    def reset(self):
        self.i_term = 0.0
        self.filter = 0.0
        self.last_err = None


class Kalman:
    """Constant-position (random-walk) Kalman filter for a 3D position, with
    innovation-gated adaptive process noise.

    A constant-velocity state was considered and rejected: this filter tracks a
    target's position in a frame that can itself rotate abruptly (e.g. a frame
    fixed to a robot that turns quickly), so "velocity" in that frame is not a
    smooth, extrapolatable quantity - during a fast turn it's dominated by the
    turn rate, not by how the tracked person is actually walking. A CV model would
    extrapolate stale pre-turn velocity straight through the event we want to
    react to.

    Instead the state stays position-only, but each step checks the normalized
    innovation (how far the new measurement is from the prediction, relative to
    the filter's own uncertainty). A small innovation means normal sensor noise
    and gets smoothed as usual. A large one means a genuine abrupt change (robot
    turned, person changed direction) - the covariance is inflated before the
    gain is computed so the correction closes most of the gap in one or two
    cycles instead of several seconds of exponential creep.

    q is the process noise rate (m^2/s, applied as Q*dt so it scales correctly
    with the actual elapsed time between updates). r is the measurement noise
    variance (m^2). gate_threshold is compared against the normalized innovation
    squared (~chi-square, 3 dof; 9.0 is roughly the 97th percentile) and
    max_inflation caps how far a single outlier can blow up the covariance.
    """

    def __init__(self, x0, P0, q, r, gate_threshold=9.0, max_inflation=200.0):
        self.x = x0
        self.x_cov = P0
        self.H = np.eye(3)
        self.Q = q * np.eye(3)
        self.R = r * np.eye(3)
        self.gate_threshold = gate_threshold
        self.max_inflation = max_inflation

    def set_initial(self, x0, P0):
        self.x = x0
        self.x_cov = P0

    def predict(self, dt):
        # A = I (no motion model); process noise still accrues with elapsed time
        self.x_cov = self.x_cov + self.Q * dt
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

    def step(self, measurement, dt):
        """predict then correct (the correct KF order - the gain must be computed
        from the predicted, not the stale pre-predict, covariance), with
        innovation gating so large genuine jumps are absorbed in one step instead
        of over several seconds of smoothing"""
        self.predict(dt)
        innovation = measurement - np.matmul(self.H, self.x)
        S = np.matmul(np.matmul(self.H, self.x_cov), self.H.T) + self.R
        d2 = float(np.matmul(np.matmul(innovation.T, np.linalg.inv(S)), innovation))
        if d2 > self.gate_threshold:
            factor = min(d2 / self.gate_threshold, self.max_inflation)
            self.x_cov = self.x_cov * factor
        return self.correct(measurement)


def declare_kalman_parameters(node, prefix="kalman", q=1.0, r=1.0, gate_threshold=9.0, max_inflation=200.0):
    """declare a Kalman filter's tuning knobs as ROS parameters and return their
    current values (from the param server / launch overrides) as a dict, ready to
    pass into Kalman(...). Pair with attach_kalman_param_callback to make them
    tunable at runtime via `ros2 param set`."""
    node.declare_parameter(f"{prefix}_q", q)
    node.declare_parameter(f"{prefix}_r", r)
    node.declare_parameter(f"{prefix}_gate_threshold", gate_threshold)
    node.declare_parameter(f"{prefix}_max_inflation", max_inflation)
    return {
        "q": node.get_parameter(f"{prefix}_q").value,
        "r": node.get_parameter(f"{prefix}_r").value,
        "gate_threshold": node.get_parameter(f"{prefix}_gate_threshold").value,
        "max_inflation": node.get_parameter(f"{prefix}_max_inflation").value,
    }


def attach_kalman_param_callback(node, kalman, prefix="kalman"):
    """update a Kalman filter's tuning in place whenever its ROS parameters change
    at runtime, so it can be retuned in the field (e.g. `ros2 param set <node>
    kalman_q 5.0`) without restarting the node"""

    def cb(params):
        for p in params:
            if p.name == f"{prefix}_q":
                kalman.Q = p.value * np.eye(3)
            elif p.name == f"{prefix}_r":
                kalman.R = p.value * np.eye(3)
            elif p.name == f"{prefix}_gate_threshold":
                kalman.gate_threshold = p.value
            elif p.name == f"{prefix}_max_inflation":
                kalman.max_inflation = p.value
        return SetParametersResult(successful=True)

    node.add_on_set_parameters_callback(cb)


def get_angle(Va, Vb, Vn):
    """returns oriented angle of rotation from Va to Vb"""
    # https://stackoverflow.com/a/33920320
    return float(
        np.arctan2(np.matmul(np.cross(Va, Vb, axis=0).T, Vn), np.matmul(Va.T, Vb))
    )


def get_transform(node, tf_buffer, tf_from, tf_to, out="matrix", time=None, dur=0.1):
    """returns the latest transformation between the given frames
    the result of multiplying point in frame tf_to by the output matrix is in the frame tf_from

    :param node: the rclpy Node (used for logging)
    :param tf_buffer: the tf2_ros Buffer to query
    :param tf_from: find transform from this frame
    :param tf_to: find transform to this frame
    :param out: the return type
                - 'matrix' - returns numpy array with the tf matrix
                - 'tf' - returns TransformStamped
    :param time: the desired timestamp of the transform (rclpy Time)
    :param dur: the timeout of the lookup (float)
    :return: as selected by out parameter or None in case of tf2 exception
                - only ConnectivityException is logged
    """
    if time is None:
        tf_time = rclpy.time.Time()
    else:
        if not isinstance(time, rclpy.time.Time):
            raise TypeError("parameter time has to be rclpy Time")
        tf_time = time

    try:
        t = tf_buffer.lookup_transform(tf_from, tf_to, tf_time, Duration(seconds=dur))
    except (LookupException, ExtrapolationException):
        return None
    except ConnectivityException as ex:
        node.get_logger().error(str(ex))
        return None

    # return the selected type
    if out == "matrix":
        return ros2_numpy.numpify(t.transform)
    elif out == "tf":
        return t
    else:
        raise ValueError("argument out should be 'matrix' or 'tf'")
