# https://atsushisakai.github.io/PythonRobotics/modules/path_tracking/stanley_control/stanley_control.html
from math import pi

import numpy as np
import rospy


class StanleyController:
    def __init__(self, traj_d, stanley_k=0.1, wheelbase=1.25):
        """
        Parameters
        ----------
        traj_d : desired_traj namedtuple
            contains x, y and yaw
        stanley_k : float, optional
            stanley portional gain, by default 0.5
        wheelbase : float, optional
            vehicle wheelbase, by default 1.25
        """
        self.x = traj_d.cx
        self.y = traj_d.cy
        self.yaw = traj_d.cyaw
        self.k = stanley_k
        self.c = 0.2
        self.wheelbase = wheelbase

    def update_trajectory(self, path):
        self.x = path.x
        self.y = path.y
        self.yaw = path.yaw

    def calc_target_index(self, state):
        """Retrieve the waypoint which is closest to the front axle.

        Parameters
        ----------
        state: namedtuple
            State of the vehicle including x, y, yaw, vel

        Returns
        -------
        target_idx: int
            Target waypoint index
        front_axle_error: float
            The distance between the target waypoint and the front axle
        """
        # Calc front axle position
        fx = state.x + self.wheelbase * np.cos(state.yaw)
        fy = state.y + self.wheelbase * np.sin(state.yaw)
        # Search the nearest waypoint based the Euclidean distance
        dx = fx - self.x
        dy = fy - self.y
        d = np.hypot(dx, dy)
        target_idx = np.argmin(d)
        # Project RMS error onto front axle vector
        front_axle_vec = [-np.cos(state.yaw + pi / 2),
                          -np.sin(state.yaw + pi / 2)]
        front_axle_error = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)
        # rospy.loginfo(
        #     f"the target position is x: {self.x[target_idx]} y: {self.y[target_idx]}")
        return target_idx, front_axle_error

    def stanley_control(self, state):
        """Generating the steering angle based on the current vehicle state and target waypoint

        Parameters
        ----------
        state : namedtuple
            State of the vehicle including x, y, yaw, vel
        last_target_idx : int
            Last waypoint index

        Returns
        -------
        delta: float
            The desired steering angle
        current_target_idx: int
            Current waypoint index
        """
        current_target_idx, front_axle_error = self.calc_target_index(state)

        # theta_e corrects the heading error
        theta_e = self.normalize_angle(
            self.yaw[current_target_idx] - state.yaw)
        # theta_d corrects the cross track error

        theta_d = np.arctan2(self.k * front_axle_error,
                             np.maximum(state.vel, 0.2))
        # Steering control
        delta = theta_e + theta_d
        # difference between actual steer angle and desired steering angle
        diff = delta - state.steering_angle
        # add the steering angle compensate
        # delta = -0.8 * (delta + self.c * diff) # linear speed is 0.4 m/s

        delta = -0.85 * (delta + self.c * diff)  # linear speed is 0.45 m/s

        return delta, current_target_idx

    @staticmethod
    def normalize_angle(angle):
        """
        Normalize an angle to [-pi, pi].

        :param angle: (float)
        :return: (float) Angle in radian in [-pi, pi]
        """
        while angle > pi:
            angle -= 2.0 * pi

        while angle < -pi:
            angle += 2.0 * pi

        return angle
