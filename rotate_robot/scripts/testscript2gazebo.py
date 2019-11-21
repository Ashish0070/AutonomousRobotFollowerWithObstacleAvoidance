#!/usr/bin/env python
import rospy
import roslib
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler 
from geometry_msgs.msg import Twist
import math
import sys
import time
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

roll = pitch = yaw = 0.0
target = -45
kP = 0.5
m = 0
l = -45
mat = np.zeros([1, 10])
deg = np.zeros([1, 10])
currentx = 0
angle = 0
c = actionlib.SimpleActionClient('move_base',MoveBaseAction)
bridge = CvBridge()
def get_rotation (msg):
	global roll, pitch, yaw
	orientation_q = msg.pose.pose.orientation
	orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
	(roll, pitch, yaw) = euler_from_quaternion (orientation_list)
	#print yaw

def callback(depth_data):
	depth_image = bridge.imgmsg_to_cv2(depth_data, "32FC1")
	depth_array = np.array(depth_image, dtype=np.float32)
	u = depth_data.width/2
	v = depth_data.height/2
	va = depth_array[u,v]
	storer(st = va)
	sub_once.unregister()

def storer(st):
	print st
	mat[0,m] = st
	deg[0,m] = target
	mat[np.isnan(mat)] = 0
	mat[np.isneginf(mat)] = 0
	mat[np.isinf(mat)] = 0

def posit(msg):
	currentx = msg.pose.pose.position.x
	currenty = msg.pose.pose.position.y
	currentz = msg.pose.pose.position.z
	sub_once2.unregister()

global sub_once

global sub_once2

rospy.init_node('rotate_robot')
sub = rospy.Subscriber ('/odom', Odometry, get_rotation)
pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
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
		if (m<10):
			sub_once = rospy.Subscriber("/camera/depth/image_raw", Image, callback)	
		time.sleep(3)
		turn = mat
		print turn
		target=target+l
		m = m+1
		if (m==4):
			target=0
			l=+45
		if (m==9):
			target=0
		if (m==10):
			maximum = max(turn[0,:])
			print maximum
			index_of_maximum = np.where(turn == maximum)
			degree = deg[index_of_maximum]
			print degree
			target = degree
			cosine = math.cos(math.radians(degree))
			sine = math.sin(math.radians(degree))
			print cosine
			print sine
		if (m==11):
			quat = 	quaternion_from_euler (roll, pitch,yaw)
			print quat
			sub_once2 = odom_sub = rospy.Subscriber('/odom', Odometry, posit)
			goal_x = currentx + maximum - 0.35
			print goal_x
			goal = MoveBaseGoal()
			goal.target_pose.pose.position.x =(goal_x*cosine)
			goal.target_pose.pose.position.y = (goal_x*sine)
			goal.target_pose.pose.orientation.x = quat[0]
			goal.target_pose.pose.orientation.y = quat[1]
			goal.target_pose.pose.orientation.z = quat[2]
			goal.target_pose.pose.orientation.w = quat[3]
			goal.target_pose.header.frame_id = "map"
			goal.target_pose.header.stamp = rospy.Time()
			c.send_goal(goal)
			wait = c.wait_for_result()
			#return c.get_result()
			sys.exit()
