#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np


class Trajectory2:
    def __init__(self, node_name):
        rospy.init_node(node_name, anonymous=True)

        # Init subscribers to retrieve position and heading from NovAtel RTK
        self.odom_sub = rospy.Subscriber("odom",
                                         Odometry, self.odom_callback)

        # Init the desired trajectory
        self.desired_traj_msg = Path()
        self.desired_traj_msg.header.frame_id = 'map'
        self.desired_traj_pub = rospy.Publisher(
            "/Desired_trajectory", Path, queue_size=10)

        # Init the publisher of path message
        self.traj_msg = Path()
        self.traj_msg.header.frame_id = 'map'
        self.traj_pub = rospy.Publisher(
            "/actual_trajectory", Path, queue_size=10)

        # load the path
        self.fp = rospy.get_param("~path_fp")
        self.load_path(self.fp)

        self.rate = rospy.Rate(int(rospy.get_param("~publish_rate")))
        self.origin = [0, 0, 0]


    def load_path(self, filepath):
        """Load the desired trajectory tractor follows

        Parameters
        ----------
        filepath : String
            Filepath of the tractor desired trajectory

        """
        trajectory = np.loadtxt(filepath, skiprows=2, delimiter=',')
        x_traj, y_traj = trajectory[:, 0], trajectory[:, 1]

        for x, y in zip(x_traj, y_traj):
            pose = PoseStamped()
            pose.pose.position.x = x
            pose.pose.position.y = y
            self.desired_traj_msg.poses.append(pose)

    def odom_callback(self, data):
        """Get the current tractor trajectory 

        Parameters
        ----------
        data : odom
            Current tractor state
        """
        if data.pose.pose.position.x == 0:
            pass
        else:
            pose = PoseStamped()
            # Calibrate the origin
            pose.pose = data.pose.pose
            # Add the current pose to the trajectory
            self.traj_msg.poses.append(pose)

    def publish_trajectory(self):
        """Publish the desired trajectory and the current tractor trajectory
        """
        while not rospy.is_shutdown():
            # Publish trajectory
            self.desired_traj_pub.publish(self.desired_traj_msg)
            self.traj_pub.publish(self.traj_msg)
            self.rate.sleep()

    def shutdown(self):
        rospy.loginfo("Shutting down trajectory node!")


if __name__ == "__main__":
    try:
        node = Trajectory2("trajectory_node")
        node.publish_trajectory()
    except rospy.ROSInterruptException:
        pass

    rospy.loginfo("Exiting trajectory node!")
