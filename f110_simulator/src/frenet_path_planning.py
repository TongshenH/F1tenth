import copy
from typing import List

from numpy import ndarray
import math

import rospy
import squaternion as quat
from params import *
from Path import FrenetPath
from sensor_msgs.msg import LaserScan
from QuarticPolynomial import QuarticPolynomial
from obb_collision_detection import Obstacle, collision_check
from QuinticPolynomialsPlanner.quintic_polynomials_planner import \
    QuinticPolynomial
from src.State import State
from nav_msgs.msg import Odometry


class FrenetPathPlanning:
    """Frenet path planning algorithm
    """
    def __init__(self, traj_d):
        self.traj_d = traj_d
        self.road_width = None

        # Init vehicle parameters
        self.x = 0
        self.y = 0
        self.yaw = 0

        # Init max gap parameters
        self.max_gap_min = None
        self.max_gap_max = None
        self.max_gap_sub = rospy.Subscriber('/max_gap', LaserScan, self.max_gap_callback)
        # Subscribe the odom message
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
    def max_gap_callback(self, data):
        """Get the max gap top

        Parameters
        ----------
        data : Laserscan
            Contain the state of the vehicle including position, orientation and vel
        """
        
        self.max_gap_max = data.angle_max
        self.max_gap_min = data.angle_min

    def odom_callback(self, data):
        """Get the vehicle_odom topic

        Parameters
        ----------
        data : odom
            Contain the state of the vehicle including position, orientation and vel
        """
        # Get the position and orientation of the vehicle
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        q = data.pose.pose.orientation
        q2 = quat.Quaternion(q.w, q.x, q.y, q.z)
        self.yaw = q2.to_euler()[2]
        

    def frenet_optimal_planning(self, state: State, obs: ndarray):
        """find the optimal local path based on the frenet algorithm

        Parameters
        ----------
        state :
            state of the vehicle.
        obs :
            All obstacles in front of the f1tenth. The shape is n x 6, where n is the number of obstacles.

        Returns
        -------
        FrenetPath
            return the best path in the Frenet coordinate
        """
        fplist = self.calc_frenet_paths(state)
        fplist = self.frenet2cart(fplist)
        print("check the path")
        fplist = self.check_paths(fplist, state)

        # find minimum cost path
        min_cost = float("inf")
        best_path = None
        for fp in fplist:
            if min_cost >= fp.cf:
                min_cost = fp.cf
                best_path = fp
        if best_path is None:
            print("can't find any path!!!!!")
            return None
        return best_path, fplist

    def check_paths(self, fplist: List, state) -> List:
        """Delete the paths not in the max gap

        Parameters
        ----------
        fplist :
            Contains paths in the FrenetPath type

        Returns
        -------
        ok_ind:
            Contains paths that satisfy constraints
        """
        ok_ind = []
        for i, _ in enumerate(fplist):
            traversable = self.traversable_check(fplist[i], state)
            if traversable:
                ok_ind.append(fplist[i])
            elif not traversable:
                continue
        return ok_ind
    
    def traversable_check(self, fp, state):
        """Check each position whether in the max gap ranges
        Args:
            fp (list): the position of each frenet path

        Returns:
            whether it is traversable
        """
        for i in range(len(fp.x)):
            ix, iy = fp.x[i], fp.y[i]
            angle = np.arctan(ix-state.x/iy-state.y)
            print("yaw", self.yaw)
            print("gap_min", self.max_gap_min)
            min_angle = self.max_gap_min - state.yaw
            max_angle = self.max_gap_max - state.yaw

            if min_angle <= angle <= max_angle:
                continue
            else:
                return False
        return True

    @staticmethod
    def calc_frenet_paths(state: State) -> List:
        """calculate the local path based on the state

        Parameters
        ----------
        state : state
            state of the vehicle

        Returns
        -------
        listntic polynomial
            Include a list contains multiple alternative paths in Frenet Path format
        """
        frenet_paths = []

        # generate path to each offset goal
        for di in np.arange(-MAX_ROAD_WIDTH / 2, MAX_ROAD_WIDTH / 2 + D_ROAD_W, D_ROAD_W):
            # Lateral motion planning
            for Ti in np.arange(MIN_T, MAX_T, DT):
                fp = FrenetPath()
                lat_qp = QuinticPolynomial(
                    state.d, state.dd, state.ddd, di, 0.0, 0.0, Ti)
                fp.t = [t for t in np.arange(0.0, Ti, DT)]
                fp.d = [lat_qp.calc_point(t) for t in fp.t]
                fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
                fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
                fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

                # Longitudinal motion planning (Velocity keeping)
                for tv in np.arange(TARGET_SPEED - D_T_S * N_S_SAMPLE,
                                    TARGET_SPEED, D_T_S):
                    tfp = copy.deepcopy(fp)
                    lon_qp = QuarticPolynomial(
                        state.s, state.v, state.a, tv, 0.0, Ti)

                    tfp.s = [lon_qp.calc_point(t) for t in fp.t]
                    tfp.s_d = [lon_qp.calc_first_derivative(
                        t) for t in fp.t]
                    tfp.s_dd = [lon_qp.calc_second_derivative(
                        t) for t in fp.t]
                    tfp.s_ddd = [lon_qp.calc_third_derivative(
                        t) for t in fp.t]

                    Jp = sum(np.power(tfp.d_ddd, 2))  # square of lateral jerk
                    Js = sum(np.power(tfp.s_ddd, 2))  # square of longitudinal jerk

                    # square of diff from target speed
                    ds = (TARGET_SPEED - tfp.s_d[-1]) ** 2

                    # Calculate the cost functional
                    tfp.cd = K_J * Jp + K_T * Ti + K_D * sum(abs(d) for d in fp.d)   # Cost function of the lateral
                    tfp.cv = K_J * Js + K_T * Ti / tv + K_D * ds ** 2      # Cost function of the longitudinal
                    tfp.cv = 0
                    tfp.cf = K_LAT * tfp.cd + K_LON * tfp.cv

                    frenet_paths.append(tfp)

        return frenet_paths

    def frenet2cart(self, fplist: List) -> List:
        """convert the path in frenet frame to the inertial frame

        Parameters
        ----------
        fplist : list
            a list contains multiple alternative paths of FrenetPath class in frenet frame

        Returns
        -------
        fplist
            a list contains multiple alternative paths of FrenetPath class in inertial frame
        """
        for fp in fplist:
            # calc global positions
            for i in range(len(fp.s)):
                ix, iy = self.traj_d.csp.calc_position(fp.s[i])
                if ix is None:
                    break
                i_yaw = self.traj_d.csp.calc_yaw(fp.s[i])
                di = fp.d[i]
                fx = ix + di * math.cos(i_yaw + math.pi / 2.0)
                fy = iy + di * math.sin(i_yaw + math.pi / 2.0)
                fp.x.append(fx)
                fp.y.append(fy)

            # calc yaw and ds
            for i in range(len(fp.x) - 1):
                dx = fp.x[i + 1] - fp.x[i]
                dy = fp.y[i + 1] - fp.y[i]
                fp.yaw.append(math.atan2(dy, dx))
                fp.ds.append(math.hypot(dx, dy))

            fp.yaw.append(fp.yaw[-1])
            fp.ds.append(fp.ds[-1])

            # calc curvature
            for i in range(len(fp.yaw) - 1):
                fp.c.append((fp.yaw[i + 1] - fp.yaw[i]) / (fp.ds[i]) + 1e-5)

        return fplist
