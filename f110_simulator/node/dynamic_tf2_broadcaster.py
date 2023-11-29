#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry

class Transform:
    def __init__(self):
        rospy.init_node('dynamic_tf2_broadcaster')

        # define the three frame coordinates
        self.br = tf2_ros.TransformBroadcaster()
        self.t = geometry_msgs.msg.TransformStamped()
        self.t2 = geometry_msgs.msg.TransformStamped()
        # define the "odom" as the IRF, base_footprint and base_link as the f1_tenth model frame
        self.t.header.frame_id = "odom"
        self.t.child_frame_id = "base_footprint"
        self.t2.header.frame_id = "base_footprint"
        self.t2.child_frame_id = "base_link"

        # get the publish rate
        self.rate = rospy.Rate(int(rospy.get_param("~publish_rate")))

        # subscribe the vehicle odom topic
        self.odom_sub = rospy.Subscriber("/odom",
                Odometry, self.odom_callback)
        
        # Init the frame transformation value
        self.position = [0, 0, 0]
        self.quaternion = [0, 0, 0, 1]


    def odom_callback(self, data):
        """Get the base_footprint frame transformation parameters 
        from the odom topic

        data
        ----------
        data : Odometry
            Tractor state 
        """
        if data.pose.pose.position.x == 0:
            pass
        else:
            self.position = [data.pose.pose.position.x, 
                             data.pose.pose.position.y, data.pose.pose.position.z]
            self.quaternion = [data.pose.pose.orientation.x, data.pose.pose.orientation.y,
                               data.pose.pose.orientation.z, data.pose.pose.orientation.w]
        
    def static_coordinate_transform(self):
        """Get the static transformation relationship between the 
        base_footprint frame and base_link frame
        """
        self.t2.header.stamp = rospy.Time.now()
        self.t2.transform.translation.x = 0
        self.t2.transform.translation.y = 0
        self.t2.transform.translation.z = 0
        self.t2.transform.rotation.x = 0
        self.t2.transform.rotation.y = 0
        self.t2.transform.rotation.z = 0
        self.t2.transform.rotation.w = 1

        self.br.sendTransform(self.t2)

    def coordinate_transform(self):
        """Publish the static and dynamic coordinate transformation
        """
        while not rospy.is_shutdown():
            self.static_coordinate_transform()
            self.t.header.stamp = rospy.Time.now()
            self.t.transform.translation.x = self.position[0]
            self.t.transform.translation.y = self.position[1]
            self.t.transform.translation.z = self.position[2]
            self.t.transform.rotation.x = self.quaternion[0]
            self.t.transform.rotation.y = self.quaternion[1]
            self.t.transform.rotation.z = self.quaternion[2]
            self.t.transform.rotation.w = self.quaternion[3]

            self.br.sendTransform(self.t)
            self.rate.sleep()

if __name__ == '__main__':
    node = Transform()
    node.coordinate_transform()