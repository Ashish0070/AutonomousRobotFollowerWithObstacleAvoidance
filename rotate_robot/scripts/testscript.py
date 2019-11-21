#!/usr/bin/env python
import rospy
import roslib
import matlab.engine
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
target = -30
kP = 0.5
m = 0
l = -30
mat = np.zeros([3, 14])
deg = np.zeros([2, 14])
dispmat = np.zeros([3, 14])
currentx = 0
currenty = 0
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
	print(st)
	mat[0,m] = st
	mat[1,m] = st
	mat[2,m] = st
	deg[0,m] = target
	deg[1,m] = target
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
		if (m<14):
			sub_once = rospy.Subscriber("/zed/zed_node/depth/depth_registered", Image, callback)
		eng = matlab.engine.start_matlab()
		eng.simplescript2(nargout=0)
		if (m<14):
			dispmat[0,m] = eng.workspace['p']
			dispmat[1,m] = eng.workspace['q']
			dispmat[2,m] = eng.workspace['r']
			print(dispmat)
			eng.quit()
		target=target+l
		m = m+1
		if (m==6):
			target=0
			l=+30
		if (m==13):
			target=0
		if (m==14):
			if (max(dispmat[0,:]) > max(dispmat[1,:])):
				maximum = max(dispmat[0,:])
			if (max(dispmat[0,:]) < max(dispmat[1,:])):
				maximum = min(dispmat[1,:])
			index_of_maximum = np.where(dispmat == maximum)
			print(deg)
			print(maximum)
			degree = deg[index_of_maximum]
			print(degree)
			target = degree
			print(mat)
			cosine = math.cos(math.radians(degree))
			sine = math.sin(math.radians(degree))
			distance = mat[index_of_maximum]
		if (m==15):
			quat = quaternion_from_euler (roll, pitch,yaw)
			print(quat)
			sub_once2 = odom_sub = rospy.Subscriber('/odom', Odometry, posit)
			goal_x = currentx + maximum*cosine -0.10
			goal_y = currenty + maximum*sine -0.10
			print(goal_x)
			print(goal_y)
			goal = MoveBaseGoal()
			goal.target_pose.pose.position.x =(goal_x)
			goal.target_pose.pose.position.y = (goal_y)
			goal.target_pose.pose.orientation.x = quat[0]
			goal.target_pose.pose.orientation.y = quat[1]
			goal.target_pose.pose.orientation.z = quat[2]
			goal.target_pose.pose.orientation.w = quat[3]
			goal.target_pose.header.frame_id = "map"
			goal.target_pose.header.stamp = rospy.Time()
			c.send_goal(goal)
			wait = c.wait_for_result()
			m = 0
			l = -30
			target = -30
