#!/usr/bin/env python3
from __future__ import print_function
import sys
import math
import numpy as np


import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

# PID CONTROL PARAMS
kp = 1.0
kd = 0.001
ki = 0.005
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0
prev_time = 0.0

# WALL FOLLOW PARAMS
ANGLE_RANGE = 270               # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9    # meters
DESIRED_DISTANCE_LEFT = 0.85
VELOCITY = 1.5                  # meters per second
CAR_LENGTH = 1.0                # Traxxas Rally is 20 inches or 0.5 meters


class WallFollow:
    """ 
    Implement Wall Following in the F1tenth race car
    """
    def __init__(self):
        global prev_time
        prev_time = rospy.get_time()

        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size = 10)


    def getRange(self, data, angle):
        """Get a range of lidar scan range

        Args:
            data (list): lidar value from topic /scan
            angle (int): between -45 to 225 degrees, where 0 degrees is 
            directly to the right, counter clockwise is positive.

        Returns:
            float: length in meters to object with angle in lidar scan view
        """
        if angle >= -45 and angle <= 225:
            iterator = len(data) * (angle + 90) / 360
            if not np.isnan(data[int(iterator)]) and not np.isinf(data[int(iterator)]):
                return data[int(iterator)]


    def pid_control(self, error, velocity):
        """Implement the PID controller

        Args:
            error (float): error between real distance and desired distance 
            velocity (float): vehicle current velocity
        """
        global integral
        global prev_error
        global kp
        global ki
        global kd
        global prev_time
        angle = 0.0
        current_time = rospy.get_time()
        del_time = current_time - prev_time

        # Use kp, ki & kd to implement a PID controller for 
        integral += prev_error * del_time
        angle = kp * error + ki * integral + kd * (error - prev_error) / del_time
        prev_error = error
        prev_time = current_time
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = -angle

        # Implement different speed based on the steering angle
        if abs(angle) > math.radians(0) and abs(angle) <= math.radians(10):
            drive_msg.drive.speed = velocity
        elif abs(angle) > math.radians(10) and abs (angle) <= math.radians(20):
            drive_msg.drive.speed = 1.4
        else:
            drive_msg.drive.speed = 0.9
        self.drive_pub.publish(drive_msg)


    def followLeft(self, data, leftDist):
        """Follow the left wall

        Args:
            data (list): list of laser scan
            leftDist (float): the distance to the left wall

        Returns:
            float: error between desired distance and real distance 
        """
        #Follow left wall as per the algorithm 
        # Implement
        front_scan_angle = 125
        back_scan_angle = 180
        theta = math.radians(abs(front_scan_angle - back_scan_angle))
        front_scan_dist = self.getRange(data, front_scan_angle)
        back_scan_dist = self.getRange(data, back_scan_angle)
        alpha = math.atan2(front_scan_dist * math.cos(theta) - back_scan_dist, front_scan_dist * math.sin(theta))
        wall_dist = back_scan_dist * math.cos(alpha)
        ahead_wall_dist = wall_dist + CAR_LENGTH * math.sin(alpha)
        return leftDist - ahead_wall_dist


    def lidar_callback(self, data):
        error = self.followLeft(data.ranges, DESIRED_DISTANCE_LEFT)
        self.pid_control(error, VELOCITY)


def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)