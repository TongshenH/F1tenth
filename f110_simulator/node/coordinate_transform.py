#!/usr/bin/env python3
import rospy
import numpy as np

import squaternion as quat
from nav_msgs.msg import Odometry
from RTK_motion_planning.msg import Detection, BBox3d


class CoordinateTransform:
    def __init__(self):
        rospy.init_node('coordinate_transform', anonymous=True)
        self.rate = rospy.Rate(10)

        # Init the bounding box and tractor parameters
        self.L = 1.94  # The horizontal distance between the camera and GPS, Unit:m
        self.h = 0.5  # The vertical distance between the camera and GPS, Unit:m
        self.x = []
        self.y = []
        self.length = []
        self.width = []
        self.yaw = []  # Bounding box yaw
        self.id = []
        self.obs = []

        # GNSS parameters
        self.gps_x = 0
        self.gps_y = 0
        self.heading = 0  # Tractor heading

        self.obs_num = 0
        self.detection_pub = rospy.Publisher('/obstacle3', Detection, queue_size=10)
        self.detection2_pub = rospy.Publisher('/obstacle2', Detection, queue_size=10)
        self.detection_sub = rospy.Subscriber('/obstacle', Detection, self.detection_callback)
        self.detection2_sub = rospy.Subscriber('/obstacle3', Detection, self.detection2_callback)
        self.odom_sub = rospy.Subscriber("vehicle_odom", Odometry, self.odom_callback)
        self.rate = rospy.Rate(10)

    def detection_callback(self, data):
        self.x = []
        self.y = []
        self.yaw = []
        self.width = []
        self.length = []
        self.id = []
        if len(data.bbox_3d) != 0:
            for b in data.bbox_3d:
                self.x.append(b.x_center)
                self.y.append(b.y_center)
                self.length.append(b.length)
                self.width.append(b.width)
                self.yaw.append(b.yaw)
                self.id.append(b.id)
            self.obs_num = len(self.x)
        else:
            rospy.loginfo("No obstacle detected")
            self.obs_num = 0

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
        yaw_vehicle = q2.to_euler()[2]
        self.gps_x = x
        self.gps_y = y
        self.heading = yaw_vehicle

    def detection2_callback(self, data):
        """
        Get obstacles in the inertial frame
        Parameters
        ----------
        data: bounding box

        Returns
        -------
        Obstacles in the inertial frame
        """
        obs = []
        for b in data.bbox_3d:
            bounding_box = [b.x_center, b.y_center, b.yaw, b.length, b.width, b.id]
            obs.append(bounding_box)
        self.obs = np.array(obs)

    def coordinate_transform(self, x, y, yaw):
        """ Transfer the bounding box parameters from the camera (odometer) frame to the Inertial reference frame
        Parameters
        ----------
        x, y, yaw: Float
        """
        # Translate the GPS point to camera point
        cam_x = self.gps_x + self.L * np.cos(self.heading)
        cam_y = self.gps_y + self.L * np.sin(self.heading)
        # Rotate the camera frame
        x_rotate = x * np.sin(self.heading) + y * np.cos(self.heading)
        y_rotate = -x * np.cos(self.heading) + y * np.sin(self.heading)
        # Integration
        x_update = cam_x + x_rotate
        y_update = cam_y + y_rotate
        yaw_update = yaw + self.heading - np.pi / 2

        return x_update, y_update, yaw_update

    def publish_updated_coordinate(self):
        """publish the updated coordinate topic"""
        while self.gps_x == 0:
            continue

        # Update the bounding box parameters
        transDetection = Detection()
        transDetection.header.stamp = rospy.get_rostime()
        transDetection.header.frame_id = "rtk_base"
        transDetection.header.seq = 10
        transDetection2 = Detection()
        transDetection2.header.stamp = rospy.get_rostime()
        transDetection2.header.frame_id = "rtk_base2"
        transDetection2.header.seq = 10
        box_indice = []
        box_x_center = []
        box_y_center = []
        while not rospy.is_shutdown():
            for i in range(self.obs_num):

                if self.id[i] not in box_indice and abs(self.x[i]) <= 5:
                    Box = BBox3d()

                    # Camera frame to the inertial frame
                    x, y, yaw = self.coordinate_transform(self.x[i], self.y[i], self.yaw[i])
                    Box.x_center, Box.y_center = x, y
                    Box.width, Box.length = self.width[i], self.length[i]
                    Box.yaw = yaw
                    Box.id = self.id[i]

                    # Compare new obstacle x center with all recorded obstacles
                    dis = np.sqrt((np.array(box_x_center) - x) ** 2 + (np.array(box_y_center) - y) ** 2)
                    not_same_box = np.all(dis > 4)
                    if not_same_box:
                        box_x_center.append(x)
                        box_y_center.append(y)
                        box_indice.append(self.id[i])
                        transDetection.bbox_3d.append(Box)
                    else:
                        continue

                    # Update the same obstacle position, the id is from one
                elif self.id[i] in box_indice:
                    index = box_indice.index(self.id[i])
                    Box = transDetection.bbox_3d[index]
                    x, y, yaw = self.coordinate_transform(self.x[i], self.y[i], self.yaw[i])
                    Box.x_center, Box.y_center = x, y
                    Box.width, Box.length = self.width[i], self.length[i]
                    Box.yaw = yaw

            transDetection2.bbox_3d = []
            # Remove obstacle behind the vehicle
            for ob in self.obs:
                Box2 = BBox3d()
                magnitude = np.sqrt((ob[0] - self.gps_x) ** 2 + (ob[1] - self.gps_y) ** 2)
                car2ob_vector = [(ob[0] - self.gps_x) / magnitude, (ob[1] - self.gps_y) / magnitude]
                car_vector = [np.cos(self.heading), np.sin(self.heading)]
                projected = np.dot(car_vector, car2ob_vector)
                if projected < 0:
                    continue
                elif projected >= 0:
                    Box2.x_center, Box2.y_center = ob[0], ob[1]
                    Box2.width, Box2.length = ob[3], ob[4]
                    Box2.yaw = ob[2]
                    Box2.id = ob[5]
                    transDetection2.bbox_3d.append(Box2)

            self.detection_pub.publish(transDetection)
            self.detection2_pub.publish(transDetection2)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        node = CoordinateTransform()
        node.publish_updated_coordinate()
    except rospy.ROSInterruptException:
        pass

    rospy.loginfo("Exiting coordinate transfer node")