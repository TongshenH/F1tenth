#!/usr/bin/env python3
import sys
import os
import math
from collections import namedtuple

current = os.path.dirname(os.path.realpath(__file__))
parent = os.path.dirname(current)
sys.path.insert(1, parent)
sys.path.insert(0, parent + "/src")

import rospy
import subprocess
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from ackermann_msgs.msg import AckermannDriveStamped
import squaternion as quat

from src.State import State
from src.stanley_controller2 import StanleyController
from src.CubicSpline.cubic_spline_planner import calc_spline_course
from src.frenet_path_planning import FrenetPathPlanning
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
        self.vel = None
        self.state = State(self.traj_d, x=0, y=0, yaw=0, v=0)
        
        # Init Time to Collision (TTC) check parameters
        self.beams = None
        self.brake = Bool()
        self.threshold = 0.1

        # Init max gap 
        self.max_gap = []

        # Init the Stanley controller
        self.stanley_controller = StanleyController(self.traj_d, K_S, L_W)
        self.frenet_optimal_planner = FrenetPathPlanning(self.traj_d)

        # Publish frequency
        self.rate = rospy.Rate(int(rospy.get_param("~publish_rate")))

        # Init subscribers to retrieve position 
        self.odom_sub = rospy.Subscriber("odom",
                                         Odometry, self.odom_callback)
        
        # Init subscribers to retrieve max gap topic 
        self.max_gap_sub = rospy.Subscriber("/max_gap",
                                         LaserScan, self.max_gap_callback)
        
        # Init local path publisher
        self.local_path_pub = None
        self.best_local_pub = rospy.Publisher('/best_local_path', Path, queue_size=10)

        # Publish bool message
        self.brake_pub = rospy.Publisher('/brake_bool', Bool, queue_size=10)

        # Init lidar beams scan subscriber
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # Init the actual steering angle subscriber
        self.actual_steering_sub = rospy.Subscriber(
            "/rand_drive", AckermannDriveStamped,
            self.actual_steering_callback)

        # Init instances for publishing Ackermann msg
        self.ackermann_msg = AckermannDriveStamped()
        self.ackermann_pub = rospy.Publisher(
            '/drive', AckermannDriveStamped, queue_size=1)

        # Init subscribers to retrieve simulation status in the simulation mode
        if self.sim == True:
            self.player = None
            self.bag_file_path = rospy.get_param("~bag_path")
            self.sim_sub = rospy.Subscriber("/status", String, self.sim_callback)

    def max_gap_callback(self, data):
        self.max_gap = [data.angle_min, data.angle_max]

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
        # self.vel = data.twist.twist.linear.x
        self.vel = 4
        self.state.update(x, y, yaw, self.vel, self.actual_steering)
        self.state.cart2frenet()

    def scan_callback(self, scan_msg):
        """Get the lidar scan topic

        scan_msg: scan
            The list of each beam's distance to the obstacle
        """
        self.beams = []
        for i in range(len(scan_msg.ranges)):
            obs_dis = scan_msg.ranges[i]
            self.angle_increment = scan_msg.angle_increment
            self.beams.append(obs_dis)
        # Time to collision check
        self.ttc_check()

    def ttc_check(self):
        """Check the TTC
        Returns:
            binary: brake or not
        """
        for i, obs_dis in enumerate(self.beams):
            if not math.isinf(obs_dis) and not math.isnan(obs_dis):

                # Calculate Time-to-Collision
                angle_increment = self.angle_increment
                ttc_denominator = max(0.0, self.vel*
                                      math.cos(i*angle_increment-math.pi))

                if ttc_denominator != 0:
                    ttc = obs_dis / ttc_denominator
                else:
                    ttc = float('inf')
                
                # Reverse motion
                if self.vel < 0 and ttc < self.threshold:
                    self.brake = True
                    self.ackermann_msg.drive.speed = 0
                    self.ackermann_pub.publish(self.ackermann_msg)
                    self.brake_pub.publish(self.brake)

                # Forward motion
                elif self.vel >= 0 and ttc < self.threshold:
                    self.brake = True
                    self.ackermann_msg.drive.speed = 0
                    # Uncomment the line below for tuning minimum TTC
                    # print("velocity:", self.vel)
                    # print("TTC{}: {}".format(i, ttc))
                    self.ackermann_pub.publish(self.ackermann_msg)
                    self.brake_pub.publish(self.brake)

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
            path, fplist = self.frenet_optimal_planner.frenet_optimal_planning(self.state, self.max_gap)
            
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
            
            
            rospy.sleep(5)
            self.stanley_controller.update_trajectory(path)

            # stanley controller return the steering angle in radian
            steering_angle, target_idx = self.stanley_controller.stanley_control(self.state)
            steering_angle = np.clip(steering_angle, -MAX_STEER, MAX_STEER)
            traj_actual.update(self.state)

            if np.hypot(path.x[1] - self.traj_d.cx[-1], path.y[1] - self.traj_d.cy[-1]) <= 2.0:
                self.ackermann_msg.drive.speed = 0
                self.ackermann_msg.drive.steering_angle = 0
                self.ackermann_pub.publish(self.ackermann_msg)
                rospy.loginfo('the target index is {}'.format(target_idx))
                rospy.loginfo("reach the destination!")
                break

            # publish the steering angle and linear speed
            self.ackermann_msg.drive.speed = path.s_d[-1]
            self.ackermann_msg.drive.steering_angle = -1 * (steering_angle)
            self.ackermann_pub.publish(self.ackermann_msg)
            self.rate.sleep()

    def shutdown(self):
        rospy.loginfo("Shutting down!")
        self.ackermann_msg.drive.speed = 0
        self.ackermann_msg.drive.steering_angle = 0
        self.ackermann_pub.publish(self.ackermann_msg)


if __name__ == '__main__':
    try:
        node = PathTrackingNode('path_tracking_node')
        node.track()
    except rospy.ROSInterruptException:
        pass
