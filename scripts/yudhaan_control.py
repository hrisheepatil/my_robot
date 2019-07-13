#!/usr/bin/env python

import rospy
import math
import tf.transformations as tft
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class yudhaanControl:

	def __init__(self, pose):
		self.vel_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)	
                self.odom_sub = rospy.Subscriber("/mobile_base_controller/odom", Odometry, self.move_to, queue_size=10)
		self.pose = pose

	def move_to(self, msg_sub):
		x = self.pose[0] - msg_sub.pose.pose.position.x
		y = self.pose[1] - msg_sub.pose.pose.position.y 
		z = self.pose[2] - msg_sub.pose.pose.position.z
		a = self.yaw_from_quaternion(msg_sub.pose.pose.orientation)
		print(a)
		msg_pub = Twist()
		msg_pub.angular.z = self.pose[3]
		self.vel_pub.publish(msg_pub)

	def yaw_from_quaternion(self, q):
        	m = tft.quaternion_matrix([q.x, q.y, q.z, q.w])
        	return math.atan2(m[1, 0], m[0, 0])

if __name__ == '__main__':
	rospy.init_node('yudhaan_control', anonymous=True)
	yc = yudhaanControl([0.0, 0.0, 0.0, 0.1])
	rospy.spin()
