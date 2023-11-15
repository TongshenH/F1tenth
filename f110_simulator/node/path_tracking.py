#!/usr/bin/env python3
import sys
import os
from collections import namedtuple

current = os.path.dirname(os.path.realpath(__file__))
parent = os.path.dirname(current)
sys.path.insert(1, parent + "/src")

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from ackermann_msgs.msg import AckermannDriveStamped
import squaternion as quat

from stanlley_controller import StanleyController
from CubicSpline.cubic_spline_planner import calc_spline_course

State = namedtuple("State", "x, y, yaw, vel, steering_angle")
pi = 3.1415926


class PathTrackingNode:
    def __init__(self, node_name):
        # Init the self.state of the tractor
        rospy.init_node(node_name, anonymous=True)

        # Register shutdown hooks
        rospy.on_shutdown(self.shutdown)

        # Init the vehicle state
        self.actual_steering = 0
        self.state = State(0, 0, 0, 0, 0)
        self.x_traj = []
        self.y_traj = []

        # Get the max steering angle
        self.max_steer = np.radians(rospy.get_param("~max_steer"))

        # Load designed path
        self.fp = rospy.get_param("~path_fp")
        self.path = self.load_path(self.fp)
        self.path_len = len(self.path)

        # Init the Stanley controller
        self.Length = float(rospy.get_param("~Length"))
        self.W = float(rospy.get_param("~W"))
        stanley_k = float(rospy.get_param("~stanley_k"))
        wheelbase = float(rospy.get_param("~wheelbase"))
        self.stanley_controller = StanleyController(
            self.path, stanley_k, wheelbase)

        # Publish frequency
        self.rate = rospy.Rate(int(rospy.get_param("~publish_rate")))

        # Init subscribers to retrieve position and heading from NovAtel RTK
        self.odom_sub = rospy.Subscriber("/odom",
                                         Odometry, self.odom_callback)

        # Init the actual steering angle subscriber
        self.actual_steering_sub = rospy.Subscriber(
            "/rand_drive", AckermannDriveStamped,
            self.actual_steering_callback)

        # Init instances for publishing Ackermann msg
        self.linear_speed = float(rospy.get_param("~linear_speed"))
        self.drive_msg = AckermannDriveStamped()
        self.ackermman_pub = rospy.Publisher(
            '/drive', AckermannDriveStamped, queue_size=1)

        self.tracking_point = PointStamped()
        self.tracking_point.header.frame_id = "map"
        self.tracking_point.header.stamp = rospy.Time.now()
        self.tracking_point_pub = rospy.Publisher("/tracking_point", PointStamped, queue_size=10)

    def actual_steering_callback(self, data):
        """Get the actual steering angle topic

        Parameters
        ----------
        data : AckermannDrive
            Actual steering angle in radian
        """
        self.actual_steering = data.drive.steering_angle

    def odom_callback(self, data):
        """Get the vehicle_odom topic

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
        vel = data.twist.twist.linear.x
        self.state = State(x, y, yaw, vel, self.actual_steering)

    def load_path(self, filepath):
        """Load the desired path from a given filepath"""
        trajectory = np.loadtxt(filepath, skiprows=2, delimiter=',')
        x_path, y_path = trajectory[:, 0], trajectory[:, 1]
        cx, cy, cyaw, *_ = calc_spline_course(x_path, y_path, ds=0.1)
        return np.array([cx, cy, cyaw]).T

    def track(self):
        """Follow the path"""

        target_idx, _, _, _ = self.stanley_controller.calc_target_index(self.state)

        while not rospy.is_shutdown():
            if target_idx < self.path_len:
                # load the trajectory
                self.x_traj.append(self.state.x)
                self.y_traj.append(self.state.y)

                # rospy.loginfo(f"target idx is {target_idx}")
                # stanley controller return the steering angle in radian
                steering_angle, target_idx = self.stanley_controller.stanley_control(
                    self.state, target_idx)
                steering_angle = np.clip(
                    steering_angle, -self.max_steer, self.max_steer)
                linear_speed = self.linear_speed
                self.tracking_point.point.x = self.path[:, 0][target_idx]
                self.tracking_point.point.y = self.path[:, 1][target_idx]
                # Visualize the tracking point
                self.tracking_point_pub.publish(self.tracking_point)

                # output the steering angle to the screen
                rospy.loginfo("Update the steering angle {}!".format(
                    np.degrees(steering_angle)))
                # rospy.loginfo("Tractor position is x: {} y:{} yaw:{}".format(
                # self.state.x, self.state.y, self.state.yaw))

            else:
                steering_angle = 0
                linear_speed = 0
                rospy.loginfo('the target index is {}'.format(target_idx))
                rospy.loginfo("reach the destination!")
            # publish the steering angle and libnear speed
            self.drive_msg.drive.speed = linear_speed
            self.drive_msg.drive.steering_angle = -1 * steering_angle
            self.ackermman_pub.publish(self.drive_msg)
            self.rate.sleep()

    def shutdown(self):
        rospy.loginfo("Shutting down!")
        self.drive_msg.drive.speed = 0
        self.drive_msg.drive.steering_angle = 0
        self.ackermman_pub.publish(self.drive_msg)


if __name__ == '__main__':
    try:
        node = PathTrackingNode('path_tracking_node')
        node.track()
    except rospy.ROSInterruptException:
        pass