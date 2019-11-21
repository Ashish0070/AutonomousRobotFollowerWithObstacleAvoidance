#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry

def callback(msg):
	print msg.pose.pose.position.x

rospy.init_node('check_odom')
odom_sub = rospy.Subscriber('/odom', Odometry , callback)

rospy.spin()
