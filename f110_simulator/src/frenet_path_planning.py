import copy
from typing import List

from numpy import ndarray
import math

from params import *
from Path import FrenetPath
from QuarticPolynomial import QuarticPolynomial
from obb_collision_detection import Obstacle, collision_check
from QuinticPolynomialsPlanner.quintic_polynomials_planner import \
    QuinticPolynomial
from src.State import State


class FrenetPathPlanning:
    """Frenet path planning algorithm
    """
    def __init__(self, traj_d):
        self.traj_d = traj_d
        self.obs = []
        self.det_range = 0
        self.num_obb = 0

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
        self.obs = obs
        fplist = self.calc_frenet_paths(state)
        fplist = self.frenet2cart(fplist)
        if len(self.obs) != 0:
            print("check the path")
            fplist = self.check_paths(fplist)

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
        return best_path, self.det_range, self.num_obb

    def check_paths(self, fplist: List) -> List:
        """Delete the paths with collision and large acceleration

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
            collision, d = self.obb_collision_detection(fplist[i])
            if any([v > MAX_SPEED for v in fplist[i].s_d]):  # Max speed check
                print("exceed max speed")
                continue
            elif any([abs(a) > MAX_ACCEL for a in fplist[i].s_dd]):  # Max accel check
                print("exceed max acceleration")
                continue
            elif not collision:
                continue
            elif collision:
                if not d:
                    ok_ind.append(fplist[i])
                elif d:
                    fplist[i].cf = fplist[i].cf + K_OBS * 1 / (sum(d) / len(d))
                    ok_ind.append(fplist[i])
        return ok_ind

    def obb_collision_detection(self, fp):
        """Oriented Bounding boxes object collision detection, detect all obstacles  in the detection collision range

        Parameters
        ----------
        fp : Frenet Path

        Returns
        -------
        Collision or not: if not collision, calculate the distance d between the
        bounding box and the obstacles
        """
        d_list = []
        for ob in self.obs:
            d_min = 255
            d = ((fp.x[1] - ob[0]) ** 2 + (fp.y[1] - ob[1]) ** 2)
            self.det_range = np.sqrt((max(fp.x) - min(fp.x)) ** 2 + (max(fp.y) - min(fp.y)) ** 2)
            if d <= self.det_range:
                v1 = Obstacle(ob)
                for (ix, iy, yaw) in zip(fp.x, fp.y, fp.yaw):
                    vehicle = Obstacle([ix, iy, yaw, L_V, W_V])
                    v2 = vehicle
                    detection, d = collision_check(v1, v2)
                    if detection is True:
                        return False, None
                    if d < d_min:
                        d_min = d
                d_list.append(d_min)
                self.num_obb = len(d_list)
        return True, d_list

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
                                    TARGET_SPEED + D_T_S * N_S_SAMPLE, D_T_S):
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
                    tfp.cd = K_J * Jp + K_T * Ti + K_D * tfp.d[-1] ** 2  # Cost function of the lateral
                    tfp.cv = K_J * Js + K_T * Ti / tv + K_D * ds * 2  # Cost function of the longitudinal
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
