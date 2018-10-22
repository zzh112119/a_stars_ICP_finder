#!/usr/bin/env python

import rospy
import numpy as np
import math
import scipy

import tf
from race.msg import drive_param
from std_msgs.msg import Float64, Vector3
from geometry_msgs.msg import Quaternion

class ICP_finder:

    def __init__(self):

        #Variables and arrays that will be used in the algorithm
        self.odom_position = np.array([0,0])
        self.odom_yaw = 0
        self.Laser_data = []
        self.pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)

        laser_data_len = 1080 #HARDCODE
        increment = 0.00436331471428 #HARDCODE
        self.angles = np.array([(-2.3561899662 + increment * i) for i in range(laser_data_len)])
        self.points = np.zeros((len(ranges), 2))

        self.result = np.zeros(4)

        rospy.Subscriber("/vesc/odom/pose/pose/position", Vector3, self.callback_odom_position)
        rospy.Subscriber("/vesc/odom/pose/pose/orientation", Quaternion, self.callback_odom_orientation)
        rospy.Subscriber("/scan", LaserScan, callback_scan)

        # self.pub_tf = rospy.Publisher("/tf")

    def callback_odom_position(self, data):
        self.odom_position = np.array([data.x,data.y])

    def callback_odom_orientation(self, data):
        temp_quaternion = [data.x,data.y,data.z,data.w]
        euler = tf.transformations.euler_from_quaternion(temp_quaternion)
        self.odom_yaw = euler[2]

    def callback_scan(self, data):
        self.Laser_data = data.ranges

    #returns a 1*4 vector of x:[x,y,cos(the),sin(the)]
    def calculate_odom(self):
        return [self.odom_position[0], self.odom_position[1], np.cos(self.odom_yaw), np.sin(self.odom_yaw)]

    #calculates the [x,y] for each point in laser frame 
    def calculate_points(self):
        for i in range(len(self.Laser_data)):
            self.points[i,0] = self.Laser_data[i] * np.cos(self.angles[i])
            self.points[i,1] = self.Laser_data[i] * np.sin(self.angles[i])

    def calculation(self):
        #TODO




    #might be useful to send the result to tf frame for visualization
    def broadcast_result(self):
       br = tf.TransformBroadcaster()
       br.sendTransform((self.result[0], self.result[1], 0),
                        tf.transformations.quaternion_from_euler(0, 0, self.result[2]),
                        rospy.Time.now(),
                        result_pos,
                        "world")

# Boilerplate code to start this ROS node.
# DO NOT MODIFY!
if __name__ == '__main__':
    rospy.init_node('ICP_finder_node', anonymous=True)
    C = ICP_finder()  
    r = rospy.Rate(40)

    while not rospy.is_shutdown():
        C.broadcast_result()
        r.sleep()


