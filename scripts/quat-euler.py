#!/usr/bin/env python

#quarternions to euler angle conversion 
import rospy
import math 
import numpy as np
import yaml
import sys
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3, Point, PoseStamped
from std_msgs.msg import Float64
import pdb
import csv
from tf.msg import tfMessage
import tf
from visualization_msgs.msg import Marker


# You can define constants in Python as uppercase global names like these.
x=0
y=0
quaternion=[]
pub = rospy.Publisher('odom_array', Point, queue_size=10)
publisher = rospy.Publisher('/visualization_odom_marker', Marker, queue_size="1")
pubs = rospy.Publisher('/visualization_odom_pose', PoseStamped, queue_size="1")
msg=Point()
marker = Marker()
pose = PoseStamped()

def Position_change(data):
	global x
	global y

	qx=data.pose.pose.orientation.x
	qy=data.pose.pose.orientation.y
	qz=data.pose.pose.orientation.z
	qw=data.pose.pose.orientation.w
	x=data.pose.pose.position.x
	y=data.pose.pose.position.y
	pose.header.frame_id = "/laser"
	pose.pose.orientation.x=qx
	pose.pose.orientation.y=qy
	pose.pose.orientation.z=qz
	pose.pose.orientation.w=qw
	pose.pose.position.x=x
	pose.pose.position.y=y
	

	quaternion = (qx,qy,qz,qw)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	roll = euler[0]
	pitch = euler[1]
	yaw = 180*euler[2]/math.pi
	msg.x = x
	msg.y = y
	msg.z = 0
	marker.header.frame_id = "/laser"
	marker.pose.position.x = x
	marker.pose.position.y = y
	marker.pose.position.z = 0

	marker.type = marker.SPHERE

	marker.scale.x = 0.3 # If marker is too small in Rviz can make it bigger here
	marker.scale.y = 0.3
	marker.scale.z = 0.3
	marker.color.a = 1.0
	marker.color.r = 1.0
	marker.color.g = 1.0
	marker.color.b = 0.0



	# Publish the MarkerArray
	print("Sending marker")
	publisher.publish(marker)
	pubs.publish(pose)



	br = tf.TransformBroadcaster()
	br.sendTransform((x, y, 0),tf.transformations.quaternion_from_euler(0, 0, yaw),rospy.Time.now(),"car","world")
	pub.publish(msg)


if __name__ == '__main__':
  rospy.init_node('odom_orientation_node', anonymous = True)
  rospy.Subscriber('/vesc/odom',Odometry,Position_change)
  rospy.spin()
