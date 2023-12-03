#!/usr/bin/env python3
import rospy
import math


from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
import squaternion as quat
import numpy as np


class Safety():
    """
    The class that handles emergency braking.
    """

    def __init__(self):
        rospy.init_node(name="Safety", anonymous=True)
        
        # Init vehicle parameters
        self.ackerman = AckermannDriveStamped()
        self.brake = Bool()
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.threshold = 0.4
        self.beams = []
        self.angle_increment = None

        # Publish AckermanDriveStamp message
        self.brake_act_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        # Publish bool message
        self.brake_pub = rospy.Publisher('/brake_bool', Bool, queue_size=10)
        # Subscribe the odom message
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        # Subscribe the laser scan message
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

    def odom_callback(self, odom_msg):
        # update current speed
        self.x = odom_msg.pose.pose.position.x
        self.y = odom_msg.pose.pose.position.y
        self.vel = odom_msg.twist.twist.linear.x
        q = odom_msg.pose.pose.orientation
        q2 = quat.Quaternion(q.w, q.x, q.y, q.z)
        self.yaw = q2.to_euler()[2]

    def scan_callback(self, scan_msg):

        for i in range(len(scan_msg.ranges)):
            obs_dis = scan_msg.ranges[i]

            print(obs_dis)

            if not math.isinf(obs_dis) and not math.isnan(obs_dis):

                print("a")

                # Calculate Time-to-Collision
                angle_increment = self.angle_increment
                ttc_denominator = max(0.0, self.vel*math.cos(i*angle_increment-math.pi))

                if ttc_denominator != 0:
                    ttc = obs_dis / ttc_denominator
                else:
                    ttc = float('inf')
                
                # Reverse motion
                if self.vel < 0 and ttc < self.threshold:
                    self.brake = True
                    self.ackerman.drive.speed = 0
                    self.brake_act_pub.publish(self.ackerman)
                    self.brake_pub.publish(self.brake)

                # Forward motion
                elif self.vel > 0 and ttc < self.threshold:
                    self.brake = True
                    self.ackerman.drive.speed = 0

                    # Uncomment the line below for tuning minimum TTC
                    print("velocity:", self.vel)
                    print("TTC{}: {}".format(i, ttc))
                    self.brake_act_pub.publish(self.ackerman)
                    self.brake_pub.publish(self.brake)

            # self.angle_increment = scan_msg.angle_increment
            # self.beams.append(obs_dis)

    # def ttc_check(self):
    #     print(self.beams)
    #     for i, obs_dis in enumerate(self.beams):
    #         if not math.isinf(obs_dis) and not math.isnan(obs_dis):

    #             # Calculate Time-to-Collision
    #             angle_increment = self.angle_increment
    #             ttc_denominator = max(0.0, self.vel*math.cos(i*angle_increment-math.pi))

    #             if ttc_denominator != 0:
    #                 ttc = obs_dis / ttc_denominator
    #             else:
    #                 ttc = float('inf')
                
    #             # Reverse motion
    #             if self.vel < 0 and ttc < self.threshold:
    #                 self.brake = True
    #                 self.ackerman.drive.speed = 0
    #                 self.brake_act_pub.publish(self.ackerman)
    #                 self.brake_pub.publish(self.brake)

    #             # Forward motion
    #             elif self.vel > 0 and ttc < self.threshold:
    #                 self.brake = True
    #                 self.ackerman.drive.speed = 0
    #                 # Uncomment the line below for tuning minimum TTC
    #                 print("velocity:", self.vel)
    #                 print("TTC{}: {}".format(i, ttc))
    #                 self.brake_act_pub.publish(self.ackerman)
    #                 self.brake_pub.publish(self.brake)


if __name__ == '__main__':
    sn = Safety()
    sn.scan_callback()
