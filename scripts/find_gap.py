#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
from a_stars_gap_finding.msg import gaps
import matplotlib.pyplot as plt

import math
import numpy as np
from time import time
from sklearn.cluster import DBSCAN

class find_gap():

    def __init__(self, maxDist = 2.0, avgWindow = 5):

        self.found_gaps = gaps()
        self.gap_center = Vector3()
        self.counter = 0

        self.center_x = 0
        self.center_y = 0
        self.max_dist = maxDist

        self.averaging_xarray = np.zeros(avgWindow)
        self.averaging_yarray = np.zeros(avgWindow)
        self.cluster_counter=0

        self.gaps_pub = rospy.Publisher('lidar_gap', gaps, queue_size=1000)
        self.center_pub = rospy.Publisher('gap_center', Vector3, queue_size=1000)
        self.turn_detect = rospy.Publisher('turn_detected', Float64, queue_size=1000)
        self.frame_array=np.zeros(40)


    # Callback that receives LIDAR data on the /scan topic.
    # data: the LIDAR data, published as sensor_msgs::LaserScan
    def scan_callback(self, data):
        self.found_gaps.widths = []
        self.found_gaps.depths = []
        self.found_gaps.angles = []

        ranges = np.asarray(data.ranges)
        # Create angle features
        angles = np.arange(data.angle_min, data.angle_max, data.angle_increment)

        # Discard unusable features
        unusable_ranges = (ranges < data.range_min) | (ranges > data.range_max) 

        # Filter by angle and range thresholds
        max_angle = 0.5* math.pi
        min_angle = -max_angle
        angles_oob = (angles < min_angle) | (angles > max_angle)

        max_range = self.max_dist
        min_range = data.range_min
        ranges_oob = (ranges < min_range) | (ranges > max_range)
        
        # Cluster valid features
        start = time()
        features = np.column_stack((angles, ranges))
        features = features[~(unusable_ranges | angles_oob | ranges_oob),:]

        predictions = DBSCAN(eps=0.3, min_samples=2).fit(features).labels_

        #Find the number of separate regions found by seeing how many different numbers there are

        obstacleIndices = set(predictions) #Added

        self.frame_array=np.append(self.frame_array,len(obstacleIndices))
        self.frame_array=np.delete(self.frame_array,0)

        a= np.where(self.frame_array>2)
        b= np.where(self.frame_array<=2)

        #print("a=",len(a[0]))
        #print("b=",len(b[0]))
        
        if(len(a[0])>28):
            self.counter=1
            #publish

        elif(len(b[0])>32):
            counter_msg=Float64()
            counter_msg.data=self.counter
            if(self.counter==0):
                self.turn_detect.publish(counter_msg)
                #print(self.cluster_counter)
                #print(self.counter)           
            elif(self.counter==1):
                self.turn_detect.publish(counter_msg)
                self.cluster_counter+=1
                #print(self.counter)
                self.counter=0



        if (len(obstacleIndices)>1):
            for obstacle in range(0,len(obstacleIndices)-1): 
                try:
                    wall_1=np.where(predictions==obstacle)
                    wall_1=np.array(wall_1)
                    start_gap = np.max(np.where(predictions == obstacle)) #Changed 

                    wall_2=np.where(predictions==obstacle+1)
                    end_gap = np.min(np.where(predictions == obstacle+1)) #Changed
                    wall_2=np.array(wall_2)
                    #print(wall_1[0])

                    #calculates the cartesian width of the gap
                    x_c = (features[end_gap][1] * math.cos(features[end_gap][0])) - (features[start_gap][1] * math.cos(features[start_gap][0]))
                    y_c = (features[end_gap][1] * math.sin(features[end_gap][0])) - (features[start_gap][1] * math.sin(features[start_gap][0]))
                    width = np.sqrt(np.power(x_c,2)+np.power(y_c,2))

                    #calculate the cartesians of the walls 
                    #X_wal_1 = features[wall_1[0]][1] * math.cos(features[wall_1[0]][0])

                    start_angle = features[start_gap][0]
                    start_range = features[start_gap][1]
                    end_angle = features[end_gap][0]
                    end_range = features[end_gap][1]

                    center_range = (start_range + end_range)/2.0
                    center_angle = (start_angle + end_angle)/2.0

                    self.center_x = center_range * math.cos(center_angle)
                    self.center_y = center_range * math.sin(center_angle)
                    self.gap_center = self.gap_averaging(self.center_x,self.center_y)

                    #add width
                    self.found_gaps.widths.append(width)
                    self.found_gaps.depths.append(center_range)
                    self.found_gaps.angles.append(center_angle)

                except ValueError:
                    pass

            #Now, with all of the found_gaps determine the widest one
            if(len(self.found_gaps.widths)>0):
                largestGap = np.argmax(self.found_gaps.widths)
                gap_center_ang = self.found_gaps.angles[largestGap]
                self.center_x = center_range * math.cos(gap_center_ang)
                self.center_y = center_range * math.sin(gap_center_ang)
                self.center_pub.publish(self.gap_center)

    #calculates the moving average of several datapoints 
    def moving_avg(self, data_array):
        output = (np.sum(data_array) - np.amax(data_array) - np.amin(data_array)) / (data_array.size - 2)

        return output

    #outputs the gap center with the moving average algorithm
    def gap_averaging(self, cx, cy):
        self.averaging_xarray = np.roll(self.averaging_xarray,1)
        self.averaging_yarray = np.roll(self.averaging_yarray,1)
        self.averaging_xarray[-1] = cx
        self.averaging_yarray[-1] = cy
        averaging_x = self.moving_avg(self.averaging_xarray)
        averaging_y = self.moving_avg(self.averaging_yarray)

        return Vector3(averaging_x, averaging_y, 0)

    #main loop to run
    def gap_finding(self):
        rospy.init_node('find_gap', anonymous=True)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        rate = rospy.Rate(10) # 10hz
        #print("The code is rolling!")
        while not rospy.is_shutdown():
            self.gap_center.x=self.center_x
            self.gap_center.y=self.center_y
            self.gaps_pub.publish(self.found_gaps)
            rate.sleep()

if __name__ == '__main__':
    gap = find_gap(maxDist=3.0, avgWindow=5)

    try:
        gap.gap_finding()
    except rospy.ROSInterruptException:
        pass

######################################################################################################################################3
# Boilerplate code to start this ROS node.
# DO NOT MODIFY!
if __name__ == '__main__':
  rospy.init_node('find_gap_node', anonymous = True)
  rospy.spin()
