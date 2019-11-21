#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler 
from geometry_msgs.msg import Twist
import math
import sys
import time

roll = pitch = yaw = 0.0
target = -45
kP = 0.5
m = 0
l = -45
def get_rotation (msg):
	global roll, pitch, yaw 
	orientation_q = msg.pose.pose.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
	#print yaw

rospy.init_node('rotate_robot')
sub = rospy.Subscriber ('/odom', Odometry, get_rotation)
pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
r = rospy.Rate(10)
command = Twist()

while not rospy.is_shutdown():
	#quat = quaternion_from_euler (roll, pitch,yaw)
	#print quat
	target_rad =target*math.pi/180
	n = (target_rad-yaw)
	command.angular.z = kP*n
	pub.publish(command)
	if (abs(n)<=0.05):
		time.sleep(5)
		target=target+l
		m = m+1
		if (m==4):
			target=0
			l=+45
		if (m==9):
			target=0
		if (m==10):
			sys.exit()
