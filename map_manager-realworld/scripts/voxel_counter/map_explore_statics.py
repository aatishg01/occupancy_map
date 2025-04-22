#!/usr/bin/env python

import rospy
from map_manager.msg import semanticObjArrayMsg, semanticObjMsg
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import csv
import os
import time
import matplotlib.pyplot as plt
import numpy as np

class Map_Explore_Statistics:
    def __init__(self):
        self.start_time = None
        self.odom = None
        self.path_length = 0
        self.view_angles_sum = 0
        self.view_angles_traversed = 0
        self.map_size = 25 * 50 * 4 * 16
        self.traversed_ratio = None
        self.pcd_ratio = None

        self.semantic_csv_file = open('semantic_objects.csv', 'w', newline='')
        self.semantic_csv_writer = csv.writer(self.semantic_csv_file)
        self.semantic_csv_writer.writerow(['path_length', 'time_since_start', 'view_angle_sum', 'view_angle_traversed', 'traversed_ratio'])

        self.voxel_csv_file = open('voxel_numbers.csv', 'w', newline='')
        self.voxel_csv_writer = csv.writer(self.voxel_csv_file)
        self.voxel_csv_writer.writerow(['path_length', 'time_since_start', 'num_of_voxels'])

        self.scores_csv_file = open('scores.csv', 'w', newline='')
        self.scores_csv_writer = csv.writer(self.scores_csv_file)
        self.scores_csv_writer.writerow(['path_length', 'time_since_start', 'traversed_ratio', 'pcd_ratio', 'final_grade'])

        self.viewangle_sub = rospy.Subscriber('/semantic_objects', semanticObjArrayMsg, self.callback)
        self.pcd_sub = rospy.Subscriber('/ugv/multi_robo_occupancy_map/explored_voxel_map', PointCloud2, self.pcd_callback)
        self.odom_sub = rospy.Subscriber('/odom_ugv', Odometry, self.odom_callback)

        while rospy.get_time() == 0.0:
            rospy.loginfo("Waiting for ROS time to be initialized...")
            rospy.sleep(0.1)

        rospy.on_shutdown(self.shutdown)

    def callback(self, data):
        current_time = time.time()
        if self.start_time is None:
            self.start_time = current_time
        time_since_start = current_time - self.start_time

        total_view_angles = len(data.semanticObjs) * 8
        total_view_angles_traversed = 0

        for semantic_obj in data.semanticObjs:
            view_angles = semantic_obj.view_angles
            total_view_angles_traversed += sum(view_angles)

        if total_view_angles != 0:
            traversed_ratio = total_view_angles_traversed / total_view_angles
            self.traversed_ratio = traversed_ratio
            row = [self.path_length, time_since_start, total_view_angles, total_view_angles_traversed, traversed_ratio]
            self.semantic_csv_writer.writerow(row)
            self.semantic_csv_file.flush()

            self.update_scores_csv(self.path_length, time_since_start)

    def pcd_callback(self, pcd):
        current_time = time.time()
        if self.start_time is None:
            self.start_time = current_time
        time_since_start = current_time - self.start_time

        pcd_ratio = pcd.width / self.map_size
        self.pcd_ratio = pcd_ratio
        row = [self.path_length, time_since_start, pcd.width]
        print(f"pcd size: {pcd.width}, map size: {self.map_size}")
        self.voxel_csv_writer.writerow(row)
        self.voxel_csv_file.flush()

        self.update_scores_csv(self.path_length, time_since_start)

    def odom_callback(self, odom):
        self.last_odom = self.odom
        self.odom = odom
        # calculate overall path length from initial orinial position to current position
        if self.start_time is not None:
            self.path_length += np.sqrt( \
                                (self.odom.pose.pose.position.x-self.last_odom.pose.pose.position.x)**2 + \
                                (self.odom.pose.pose.position.y-self.last_odom.pose.pose.position.y)**2 )
    
    # def calculate_path_length(self):
    #     if self.odom is None:
    #         return 0
    #     if self.start_time is None:
    #         return 0
    #     current_time = time.time()
    #     time_since_start = current_time - self.start_time
    #     if time_since_start == 0:
    #         return 0
    #     path_length = 0
    #     for i in range(1, len(self.odom.pose.pose.position)):
    #         path_length += (self.odom.pose.pose.position[i].x - self.odom.pose.pose.position[i-1].x)**2 + (self.odom.pose.pose.position[i].y - self.odom.pose.pose.position[i-1].y)**2
    #     return path_length
    


    def update_scores_csv(self, path_length, time_since_start):
        if self.traversed_ratio is not None and self.pcd_ratio is not None:
            final_grade = 0.5 * self.traversed_ratio + 0.5 * self.pcd_ratio
            row = [path_length, time_since_start, self.traversed_ratio, self.pcd_ratio, final_grade]
            self.scores_csv_writer.writerow(row)
            self.scores_csv_file.flush()
            self.traversed_ratio = None
            self.pcd_ratio = None

    def shutdown(self):
        # Plot the scores
        times = []
        traversed_ratios = []
        pcd_ratios = []
        final_grades = []

        # with open('scores.csv', 'r') as f:
        #     reader = csv.DictReader(f)
        #     for row in reader:
        #         times.append(float(row['time_since_start']))
        #         traversed_ratios.append(float(row['traversed_ratio']))
        #         pcd_ratios.append(float(row['pcd_ratio']))
        #         final_grades.append(float(row['final_grade']))

        with open('voxel_numbers.csv', 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                times.append(float(row['time_since_start']))
                pcd_ratios.append(float(row['num_of_voxels']))

        self.semantic_csv_file.close()
        self.voxel_csv_file.close()
        self.scores_csv_file.close()

        plt.figure()
        # plt.plot(times, traversed_ratios, label='Traversed Ratio')
        plt.plot(times, pcd_ratios, label='PCD Ratio')
        # plt.plot(times, final_grades, label='Final Grade')
        plt.xlabel('Time Since Start (s)')
        plt.ylabel('Ratio / Grade')
        plt.title('Map Exploration Statistics')
        plt.legend()
        plt.savefig('exploration_statistics.png')
        plt.show()

if __name__ == '__main__':
    rospy.init_node('map_explore_statistics')
    subscriber = Map_Explore_Statistics()
    rospy.spin()