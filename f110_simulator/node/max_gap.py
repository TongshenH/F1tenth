#!/usr/bin/env python3
import sys
import os 
current = os.path.dirname(os.path.realpath(__file__))
parent = os.path.dirname(current)
sys.path.insert(1, parent)
sys.path.insert(0, parent + "/src")

import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from src.gap import MaxGapState

class MaxGap:
    """Follow the gap between obstacles
    """
    def __init__(self):
        rospy.init_node("MaxGap_node")

        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        self.max_gap_pub = rospy.Publisher("/max_gap", LaserScan, queue_size=10)
        
        # Init follow the gap parameters
        self.bubble_radius = 20           # cm
        self.max_distance = 5
        self.window_size = 90             # degree
        self.max_gap_state = MaxGapState()

        # Init the lidar scan range
        self.current_proc_ranges = np.zeros((1080,), dtype=np.float32)

    def preprocess_lidar_scan(self, ranges):
        """Replace the lidar distance with the max distance by the max threshold,
        Args:
            ranges (list): the lidar range list
        """
        center_idx = int(len(ranges)/2)
        len_beams = int((self.window_size/360) * len(ranges))
        print(len(ranges))
        min_idx, max_idx = center_idx-len_beams, len_beams+center_idx
        proc_ranges = np.array(ranges, dtype=np.float32)
        self.current_proc_ranges = proc_ranges[min_idx:max_idx]

        # Filter the lidar beams out of max distance
        filter_idx = self.current_proc_ranges > self.max_distance
        self.current_proc_ranges[filter_idx] = self.max_distance

    def process_bubble(self, ranges):
        i = 0
        while i < len(ranges):
            if ranges[i] <= 1.1:
                ranges[max(0, i - self.bubble_radius):i + self.bubble_radius] = 0
                i += self.bubble_radius
            else:
                i += 1
        return ranges
    
    @staticmethod
    def max_gap(free_space_ranges):
        """Find the maxmium gap based on the free space range 

        Args:
            free_space_ranges (list): list of free space range

        Returns:
            start_i: start index of the max gap
            end_i: end index of the max gap
            max_length_ranges: lidar ranges in max gap 
        """
        split_idx = np.where(free_space_ranges == 0.0)[0]
        sp_ranges = np.split(free_space_ranges, split_idx)
        len_sp_ranges = np.array([len(x) for x in sp_ranges])
        max_idx = np.argmax(len_sp_ranges)
        # Check the max gap index whether start from the first lidar beam
        if max_idx == 0:
            start_i = 0
            end_i = len_sp_ranges[0]-1
        else:
            start_i = np.sum(len_sp_ranges[:max_idx])
            end_i = start_i + len_sp_ranges[max_idx]-1
        max_gap_ranges = sp_ranges[max_idx]
        return int(start_i), int(end_i), max_gap_ranges

    def lidar_callback(self, data):
        ranges = data.ranges
        self.preprocess_lidar_scan(ranges)
        ranges = self.process_bubble(self.current_proc_ranges)
        i_s, i_e, max_gap = self.max_gap(ranges)
        intensities = data.intensities
        angle_increment = data.angle_increment

        # Publish the max gap laser scan
        max_gap_msg = LaserScan()
        max_gap_msg.header.stamp = data.header.stamp
        max_gap_msg.header.frame_id = 'laser'
        max_gap_msg.ranges = max_gap
        max_gap_msg.range_max = data.range_max
        max_gap_msg.intensities = intensities[i_s: i_e]
        max_gap_msg.angle_increment = angle_increment
        max_gap_msg.angle_min = i_s * angle_increment - np.radians(self.window_size)
        max_gap_msg.angle_max = i_e * angle_increment - np.radians(self.window_size)

        self.max_gap_pub.publish(max_gap_msg)
        self.max_gap_state.update(max_gap_msg)

def main():
    maxgap = MaxGap()
    rospy.spin()


if __name__ == "__main__":
    main()