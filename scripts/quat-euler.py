#!/usr/bin/env python

#quarternions to euler angle conversion 
import rospy
import math 
import numpy as np
import yaml
import sys
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
import pdb
import csv
from tf.msg import tfMessage
import tf


# You can define constants in Python as uppercase global names like these.
x=0
y=0
quaternion=[]
pub = rospy.Publisher('odom_array', Vector3, queue_size=10)
msg=Vector3()

def Position_change(data):
	global x
	global y
	qx=data.pose.pose.orientation.x
	qy=data.pose.pose.orientation.y
	qz=data.pose.pose.orientation.z
	qw=data.pose.pose.orientation.w
	x=data.pose.pose.position.x
	y=data.pose.pose.position.y
	quaternion = (qx,qy,qw,qz)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	roll = euler[0]
	pitch = euler[1]
	yaw = 180*euler[2]/math.pi
	msg.x=x
	msg.y=y
	msg.z=yaw
	br = tf.TransformBroadcaster()
	br.sendTransform((x, y, 0),tf.transformations.quaternion_from_euler(0, 0, yaw),rospy.Time.now(),"car","world")
	pub.publish(msg)


if __name__ == '__main__':
  rospy.init_node('odom_orientation_node', anonymous = True)
  rospy.Subscriber('/vesc/odom',Odometry,Position_change)
  rospy.spin()
