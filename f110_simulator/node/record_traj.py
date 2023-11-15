#!/usr/bin/env python3
import rospy
import csv

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry


class DesiredTrajectory():
    """
    The Class record the trajectory(x, y) manually, the global frame is based 
    on the initial position
    """
    
    # Init the vehicle parameters
    def __init__(self):
        rospy.init_node(name="Record Trajectory", anonymous=True)
        
        # Init vehicle parameters
        self.ackerman = AckermannDriveStamped()
        self.x = []
        self.y = []
        self.yaw = 0
        self.threshold = 0.4

        # Subscribe the odom message
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        # Set the time interval to 1 second
        self.timer = rospy.Timer(rospy.Duration(1.0), self.timer_callback)

    def odom_callback(self, odom_msg):
        # Gain current speed and position
        self.pos_x = odom_msg.pose.pose.position.x
        self.pos_y = odom_msg.pose.pose.position.y
        self.vel = odom_msg.twist.twist.linear.x

    def timer_callback(self, time_msg):
        rospy.loginfo("Recording odometry data...")
        self.x.append(self.pos_x)
        self.y.append(self.pos_y)
          
    def save_to_csv(self):
        # Save collected odometry data to a CSV file
        rospy.loginfo("Saving odometry data to CSV...")

        file_path = 'desired_traj.csv'
        with open(file_path, 'w') as csv_file:
            fieldnames = ['pose_x', 'pose_y', 'yaw']
            writer = csv.writer(csv_file)
            writer.writerow(fieldnames)
            for i in range(len(self.x)):
                writer.writerow([self.x[i], self.y[i]])


def main():
    while not rospy.is_shutdown():
        record = DesiredTrajectory()
        rospy.spin()
    record.save_to_csv()
    rospy.loginfo("Shutting down...")


if __name__ == "__main__":
    main()
