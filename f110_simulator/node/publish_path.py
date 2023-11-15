#!/usr/bin/env python3
import csv

import pandas as pd
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

def publish_path():
    rospy.init_node('path_publisher')
    path_pub = rospy.Publisher('/global_path', Path, queue_size=10)
    rate = rospy.Rate(1)  # Adjust the publishing rate as needed

    path_msg = Path()
    path_msg.header.frame_id = 'map'
    path_msg.header.stamp = rospy.get_rostime()

    # Read data from CSV file and populate PoseArray
    # Replace this section with your CSV reading logic

    path = np.array(pd.read_csv("/home/tongshen/Projects/Scaled_auto_vehicle/catkin_ws/src/f110_ros/f110_simulator/trajectory"
                                "/key_points.csv", header=0))
    for point in path:
        pose = PoseStamped()
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        path_msg.poses.append(pose)

    while not rospy.is_shutdown():
        path_pub.publish(path_msg)
        rate.sleep()


if __name__ == '__main__':
    try:
        publish_path()
    except rospy.ROSInterruptException:
        pass
