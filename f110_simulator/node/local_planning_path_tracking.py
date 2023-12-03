#!/usr/bin/env python3
import sys
import os
from collections import namedtuple

current = os.path.dirname(os.path.realpath(__file__))
parent = os.path.dirname(current)
sys.path.insert(1, parent)
sys.path.insert(0, parent + "/src")
print(sys.path)

import rospy
import subprocess
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
# from novatel_oem7_msgs.msg import HEADING2
from ackermann_msgs.msg import AckermannDriveStamped
import squaternion as quat

from src.State import State
from src.stanley_controller2 import StanleyController
from src.CubicSpline.cubic_spline_planner import calc_spline_course
from src.frenet_path_planning import FrenetPathPlanning
# from RTK_motion_planning.msg import BBox3d, Detection
from src.Path import FrenetPath
from src.Visualization import visualization
from src.params import *

show_animation = False
desired_traj = namedtuple("desired_traj", "cx, cy, cyaw, ck, dck, s, csp")


class PathTrackingNode:
    def __init__(self, node_name):
        # Init the self.state of the tractor
        rospy.init_node(node_name, anonymous=True)

        # Register shutdown hooks
        rospy.on_shutdown(self.shutdown)

        # Load designed path
        self.sim = rospy.get_param("~sim")
        self.fp = rospy.get_param("~path_fp")
        self.traj_d = self.load_path(self.fp)

        # Init the vehicle state
        self.actual_steering = 0
        self.state = State(self.traj_d, x=0, y=0, yaw=0, v=0)
        self.obs = []

        # Init the Stanley controller
        self.stanley_controller = StanleyController(self.traj_d, K_S, L_W)
        self.frenet_optimal_planner = FrenetPathPlanning(self.traj_d)

        # Publish frequency
        self.rate = rospy.Rate(int(rospy.get_param("~publish_rate")))

        # Init subscribers to retrieve position and heading from NovAtel RTK
        self.odom_sub = rospy.Subscriber("odom",
                                         Odometry, self.odom_callback)
        
        # Init local path publisher
        self.local_path_pub = None
        self.best_local_pub = rospy.Publisher('/best_local_path', Path, queue_size=10)

        # Init subscribers to retrieve obstacle information
        # self.obs_sub = rospy.Subscriber("/obstacle2", Detection, self.obs_callback)

        # Init the actual steering angle subscriber
        self.actual_steering_sub = rospy.Subscriber(
            "/rand_drive", AckermannDriveStamped,
            self.actual_steering_callback)

        # Init instances for publishing Ackermann msg
        self.ackermann_msg = AckermannDriveStamped()
        self.ackermman_pub = rospy.Publisher(
            '/drive', AckermannDriveStamped, queue_size=1)

        # Init subscribers to retrieve simulation status in the simulation mode
        if self.sim == True:
            self.player = None
            self.bag_file_path = rospy.get_param("~bag_path")
            self.sim_sub = rospy.Subscriber("/status", String, self.sim_callback)

    def sim_callback(self, data):
        """Get the status when the camera finishes initialization in the simulation mode

        Parameters
        ----------
        data : String
            Camera status
        """
        bagfile = "record1.bag"
        rospy.loginfo("start to play the rosbag!")
        if data.data == "Done" and self.player is None:
            self.player = subprocess.Popen(['rosbag', 'play', bagfile], cwd=self.bag_file_path)

    def obs_callback(self, data):
        """Get the obstacle dimension, position and orientation

        Parameters
        ----------
        data : Detection.msg
            Custom created message 
        """
        obs = []
        for b in data.bbox_3d:
            bounding_box = [b.x_center, b.y_center, b.yaw, b.length, b.width, b.id]
            obs.append(bounding_box)
        self.obs = np.array(obs)
        

    def actual_steering_callback(self, data):
        """Get the actual steering angle topic

        Parameters
        ----------
        data : AckermannDrive
            Actual steering angle in radian
        """
        self.actual_steering = data.drive.steering_angle

    def odom_callback(self, data):
        """Get the vehicle odom topic

        Parameters
        ----------
        data : odom
            Contain the state of the vehicle including position, orientation and vel
        """
        # Get the position and orientation of the vehicle
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        q = data.pose.pose.orientation
        q2 = quat.Quaternion(q.w, q.x, q.y, q.z)
        yaw = q2.to_euler()[2]
        # Get the vel of the vehicle
        # vel = data.twist.twist.linear.x
        vel = 4.0
        self.state.update(x, y, yaw, vel, self.actual_steering)
        self.state.cart2frenet()

    def load_path(self, filepath):
        """Load the predefined path

        Parameters
        ----------
        filepath : string
            Paths that saved pre-defined trajectories

        Returns
        -------
        desired_traj namedtuple contained the predefined trajectory
        """
        trajectory = np.loadtxt(filepath, skiprows=1, delimiter=',')
        x_path, y_path = trajectory[:, 0], trajectory[:, 1]
        cx, cy, cyaw, ck, dck, s_list, csp = calc_spline_course(x_path, y_path, ds=0.1)
        traj_d = desired_traj(cx, cy, cyaw, ck, dck, s_list, csp)
        return traj_d

    def track(self):
        """Follow the path"""
        # get the first target waypoint
        traj_actual = FrenetPath()
        target_idx, _ = self.stanley_controller.calc_target_index(self.state)

        while not rospy.is_shutdown():
            
            local_paths = []
            path, det_range, num_obb, fplist = self.frenet_optimal_planner.frenet_optimal_planning(self.state, self.obs)
            
            # Publish the local path list
            for i in range(len(fplist)):
                self.local_path_pub = rospy.Publisher(f'/local_path{i}', Path, queue_size=10)
                local_paths.append(self.local_path_pub)
            for j, p in enumerate(fplist):
                local_path_msg = Path()
                local_path_msg.header.frame_id = 'map'
                local_path_msg.header.stamp = rospy.get_rostime()
                for i in range(len(p.x)):
                    pose = PoseStamped()
                    pose.pose.position.x = p.x[i]
                    pose.pose.position.y = p.y[i]
                    local_path_msg.poses.append(pose)
                local_paths[j].publish(local_path_msg)

            if path is None:
                break
            
            # Publish the best path
            best_path_msg = Path()
            best_path_msg.header.frame_id = 'map'
            best_path_msg.header.stamp = rospy.get_rostime()
            for i in range(len(path.x)):
                    pose = PoseStamped()
                    pose.pose.position.x = path.x[i]
                    pose.pose.position.y = path.y[i]
                    best_path_msg.poses.append(pose)
            self.best_local_pub.publish(best_path_msg)
            

            self.stanley_controller.update_trajectory(path)

            # stanley controller return the steering angle in radian
            steering_angle, target_idx = self.stanley_controller.stanley_control(self.state)
            steering_angle = np.clip(steering_angle, -MAX_STEER, MAX_STEER)
            traj_actual.update(self.state)

            if show_animation:
                visual.show_animation(self.state, path, target_idx, traj_actual, self.obs, det_range, num_obb)

            if np.hypot(path.x[1] - self.traj_d.cx[-1], path.y[1] - self.traj_d.cy[-1]) <= 2.0:
                self.ackermann_msg.drive.speed = 0
                self.ackermann_msg.drive.steering_angle = 0
                self.ackermman_pub.publish(self.ackermann_msg)
                rospy.loginfo('the target index is {}'.format(target_idx))
                rospy.loginfo("reach the destination!")
                break

            # publish the steering angle and libnear speed
            self.ackermann_msg.drive.speed = path.s_d[-1]
            self.ackermann_msg.drive.steering_angle = -1 * (steering_angle)
            self.ackermman_pub.publish(self.ackermann_msg)
            self.rate.sleep()

    def shutdown(self):
        rospy.loginfo("Shutting down!")
        self.ackermann_msg.drive.speed = 0
        self.ackermann_msg.drive.steering_angle = 0
        self.ackermman_pub.publish(self.ackermann_msg)


if __name__ == '__main__':
    try:
        node = PathTrackingNode('path_tracking_node')
        node.track()
    except rospy.ROSInterruptException:
        pass
