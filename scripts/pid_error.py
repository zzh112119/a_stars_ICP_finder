#!/usr/bin/env python

import rospy
from math import cos, sin, atan, pi
import numpy as np
import yaml
import sys
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import *
from std_msgs.msg import Float64
import pdb
import csv
from tf.msg import tfMessage
import tf

# You can define constants in Python as uppercase global names like these.
MIN_DISTANCE = 0.1
MAX_DISTANCE = 30.0
MIN_ANGLE = -45.0
MAX_ANGLE = 225.0
a = 0.0
b = 0.0
i=0.0
yaw_old=0.0
t=0
i=0
k=0
m=0
i_store = 0 
i_old = 0
c_end = 0
error=0
commanded_vel = 2 #initial velocity should be 2 m/s as the car is going in a straight line. This changes once a turn is made

#Define the publishers
pub = rospy.Publisher('pid_error', Float64, queue_size=10)
velocityPub = rospy.Publisher('command_velocity',Float64,queue_size=10)

turns_array=[]
with open('turns.txt','r') as turns_file:
    turns_reader = csv.reader(turns_file,delimiter=',')
    #print(turns_reader(1))
    for row in turns_reader:
       turns_array.append(row)
turns_array.pop()
#print(turns_array)

# data: single message from topic /scan
# angle: between -45 to 225 degrees, where 0 degrees is directly to the right
# Outputs length in meters to object with angle in lidar scan field of view
def getRange(data, angle, old_range):

  ranges = np.asarray(data.ranges)
  angle_index = (angle + 45) * 4
  if ranges[angle_index] == np.nan or ranges[angle_index] == np.inf:
    return old_range
  else:
    output_range = ranges[angle_index]
    return output_range

# data: single message from topic /scan
# desired_distance: desired distance to the left wall [meters]
# Outputs the PID error required to make the car follow the left wall.
def followLeft(data, desired_distance):
  global a,b
  L = 0.025
  desired_distance = desired_distance
  a = getRange(data, 135, a)
  b = getRange(data, 180, b)
  theta = 45 * pi / 180
  alpha = atan((a*cos(theta)-b)/(a*sin(theta)))
  current_dist  = b * cos(alpha)
  next_dist = current_dist + L * sin(alpha)
  error_t = desired_distance - current_dist

  # pass the error_t term into some function and output the next_angle and velocity
  return error_t

# data: single message from topic /scan
# desired_distance: desired distance to the right wall [meters]
# Outputs the PID error required to make the car follow the right wall.
def followRight(data, desired_distance):
  global a,b
  L = 0.025
  desired_distance = desired_distance
  a = getRange(data, 45, a)
  b = getRange(data, 0, b)
  theta = 45 * pi / 180
  alpha = atan((a*cos(theta)-b)/(a*sin(theta)))
  current_dist  = b * cos(alpha)
  next_dist = current_dist + L * sin(alpha)
  error_t = desired_distance - current_dist

  return -error_t

# data: single message from topic /scan
# Outputs the PID error required to make the car drive in the middle
# of the hallway.
def followCenter(data):
  left_error = followLeft(data,1)
  right_error = followRight(data,1)
  center_error = (left_error + right_error)/2

  return center_error

# Callback for receiving LIDAR data on the /scan topic.
# data: the LIDAR data, published as a list of distances to the wall.
def scan_callback(data):
	global error
	global commanded_vel
	desired_distance=0.8

	if(turns_array[i][0]=="center"):
		error=followCenter(data)

	else:
		error=followLeft(data,desired_distance)

	msg = Float64()
	msg.data = error
	pub.publish(msg)
	commanded_vel = float(turns_array[i][1])
	v_msg = Float64()
	v_msg.data = commanded_vel
	velocityPub.publish(v_msg)

def turn_callback(data):
	global i
	
	if data.data == 1: #turn has been detected
		i+=1

######################################################################################################################################3
# Boilerplate code to start this ROS node.
# DO NOT MODIFY!
if __name__ == '__main__':
  rospy.init_node('pid_error_node', anonymous = True)
  rospy.Subscriber("scan", LaserScan, scan_callback)
  rospy.Subscriber("turn_detected", Float64, turn_callback)
  rospy.spin()

