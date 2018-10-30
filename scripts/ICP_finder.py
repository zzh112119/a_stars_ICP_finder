#!/usr/bin/env python

import rospy
import numpy as np
from scipy.optimize import minimize

import tf
from race.msg import drive_param
from std_msgs.msg import Float64
from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg import LaserScan

class ICP_finder:

    def __init__(self):
        self.pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)
        # Variables and arrays that will be used in the algorithm
        # TODO: check if initialization is ok and values are being changed
        self.odom_position = np.zeros(2)
        self.odom_yaw = 0
        self.points = np.zeros((500, 2))
        self.points_old = np.zeros((0, 2))
        self.roto_4 = np.eye(4)
        self.roto_1 = np.zeros((4, 1))
        self.position = np.zeros(3)
        self.is_first = True
        self.is_second = False
        rospy.Subscriber("/vesc/odom/pose/pose/position", Vector3, self.callback_odom_position)
        rospy.Subscriber("/vesc/odom/pose/pose/orientation", Quaternion, self.callback_odom_orientation)
        rospy.Subscriber("/scan", LaserScan, self.callback_scan)

        # self.pub_tf = rospy.Publisher("/tf")
        self.loc_tf = rospy.Publisher('scan_match_location', Vector3, queue_size=1)

    def callback_odom_position(self, data):
        self.odom_position = np.array([data.x,data.y])

    def callback_odom_orientation(self, data):
        temp_quaternion = [data.x,data.y,data.z,data.w]
        euler = tf.transformations.euler_from_quaternion(temp_quaternion)
        self.odom_yaw = 180*euler[2]/math.pi

    # Convert ranges to points, removing bad data.
    def callback_scan(self, data):
        ranges = np.array(data.ranges)[0:500]
        n = 500#len(ranges)

        # Calculate angles.
        angle_min = data.angle_min
        angle_increment = data.angle_increment
        angles = np.array(
            [(angle_min + angle_increment * i) for i in range(n)])

        range_min = data.range_min
        range_max = data.range_max

        # Calculate old points.
        points = np.zeros((n, 2))
        for i in range(n):
            points[i, 0] = ranges[i] * np.cos(angles[i])
            points[i, 1] = ranges[i] * np.sin(angles[i])

        # Adjust bad data
        #points = points[~((ranges < range_min) | (ranges > range_max) | (ranges == np.nan) | (ranges == np.inf))]
        points[(ranges < range_min)] = range_min
	points[(ranges > range_max)] = range_max
	points[(ranges == np.inf)] = range_max
	points[(ranges == np.nan)] = 0
	self.points = points[0:500]

    def calculation(self):
        # Set odometry as first guess x0 = [t_x, t_y, cos(theta), sin(theta)]
        # TODO: check we should use odom in time 0 and then roto
        if self.is_first:
            self.points_old = self.points
            self.is_first = False
            self.is_second = True
            return
        elif self.is_second:
            x0 = np.array([self.odom_position[0], self.odom_position[1],
                           np.cos(self.odom_yaw), np.sin(self.odom_yaw)])
            x0 = x0.reshape((4, 1))
            self.is_second = False
        else:
            x0 = self.roto_1
        x = x0

        # Solve x* = argmin_x x'M x + g'x s.t. x'Wx <= 1 and reproject to = 1

        # Calculate all M_i.
        M_i_s = np.empty((len(self.points), 2, 4))
        M_i_s[:, 0, 0] = 1
        M_i_s[:, 0, 1] = 0

        M_i_s[:, 0, 2] = self.points[:, 0]
        M_i_s[:, 0, 3] = -self.points[:, 1]

        M_i_s[:, 1, 0] = 0
        M_i_s[:, 1, 1] = 1
        M_i_s[:, 1, 2] = self.points[:, 1]
        M_i_s[:, 1, 3] = self.points[:, 0]

        # Calculate M.
        M = np.zeros((4, 4))
        for M_i in M_i_s:
            M += np.matmul(np.transpose(M_i), M_i)

        # Get the points of the old scan.
        xs_old = self.points_old[:, 0]
        ys_old = self.points_old[:, 1]

        # https://docs.scipy.org/doc/scipy-0.19.1/reference/generated/scipy.optimize.minimize.html
        def optimizer(x):
            x = x.reshape((4, 1))
            return np.matmul(np.matmul(np.transpose(x), M), x) + np.matmul(
                np.transpose(g), x)

        # Guess x: while old x and new x are far, update the old x and repeat.
        k = 0
        max_k = 10
        epsilon = 1
        while True:
            # Calculate g, projecting points into old frame using guess for x.
            g = np.zeros((1, 4))

            for i, M_i in enumerate(M_i_s):
                # Project new points into old frame using guess for x.
                x_new, y_new = np.matmul(M_i, x)
                distances = np.sqrt(
                    np.square(xs_old - x_new) + np.square(ys_old - y_new))

                # Get the closest point.
                pi_i = self.points_old[np.argmin(distances)]
                pi_i = pi_i.reshape((2, 1))

                # Update g.
                g += -2 * np.matmul(np.transpose(pi_i), M_i)

            g = np.transpose(g)

            # Transform sin(theta)^2 + cos(theta)^2 <= 1 to constraint >= 0.
            cons = (
            {'type': 'ineq', 'fun': lambda x: -(x[2] ** 2 + x[3] ** 2 - 1)})

            res = minimize(optimizer, x, constraints=cons)
            x_new = res.x

            # Project onto the feasible set cos(theta)^2 + sin(theta)^2 = 1.
            intersection_polar = [1, np.arctan2(x_new[3], x_new[2])]
            x_new = np.array(
                [x_new[0], x_new[1], np.cos(intersection_polar[1]),
                 np.sin(intersection_polar[1])])

            # Check that the projection worked.
            # print(x_new[2]**2 + x_new[3]**2)

            # Project points using new guess.
            points_proj = np.array(
                [np.matmul(M_i, x_new) for i, M_i in enumerate(M_i_s)])
            xs_proj, ys_proj = points_proj[:, 0], points_proj[:, 1]
            x = x_new

	    
            guess_distance_mse = np.sqrt(sum(np.square(xs_old - xs_proj) + np.square(ys_old - ys_proj)))
            if guess_distance_mse < epsilon or k > max_k:
		print('guess distance: ')		
		print(guess_distance_mse)                
		break
            else:
                k += 1

        # Calculate position
        #print(x)
	#rospy.loginfo(x)
        roto = np.eye(4)
        roto[0, 3]  = x[0]
        roto[1, 3]  = x[1]
        roto[0, 0]  = x[2]
        roto[1, 1]  = x[2]
        roto[1, 0]  = x[3]
        roto[0, 1]  = -x[3]
        self.roto_4 = np.matmul(self.roto_4, roto)

	print('roto 4 x: ')
	print(self.roto_4[0,3])

	print('roto 4 y: ')
	print(self.roto_4[1,3])

        self.roto_1 = np.array([self.roto_4[0, 3], self.roto_4[1, 3], self.roto_4[0, 0], self.roto_4[1, 0]])
        self.roto_1 = self.roto_1.reshape((4, 1))
        M_i = np.array([[1, 0, self.position[0], -self.position[1]],
                        [0, 1, self.position[1], self.position[0]]])
        position = np.matmul(M_i, self.roto_1)
        self.position = position
	
	#print('position: ')
	#print(position)

        # TODO: check topic being published to
        self.loc_tf.publish(Vector3(position[0], position[1], 0))
        self.points_old = self.points

    # might be useful to send the result to tf frame for visualization
    def broadcast_result(self):
        br = tf.TransformBroadcaster()
        br.sendTransform((self.result[0], self.result[1], 0),
                          tf.transformations.quaternion_from_euler(0, 0, self.result[2]),
                          rospy.Time.now(),
                          self.position,
                          "world")

# Boilerplate code to start this ROS node.
# DO NOT MODIFY!
if __name__ == '__main__':
    rospy.init_node('ICP_finder_node', anonymous=True)
    C = ICP_finder()  
    r = rospy.Rate(40)

    while not rospy.is_shutdown():
        C.calculation()
        #C.broadcast_result()
        r.sleep()


