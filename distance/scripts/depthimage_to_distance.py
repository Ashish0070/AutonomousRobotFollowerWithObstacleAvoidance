#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

rospy.init_node('zed_depth', anonymous=True)
bridge = CvBridge()
def callback(depth_data):
	depth_image = bridge.imgmsg_to_cv2(depth_data, "32FC1")
	depth_array = np.array(depth_image, dtype=np.float32)
	u = depth_data.width/2
	v = depth_data.height/2
	dist=depth_array[u,v]
	print dist
rospy.Subscriber("/zed/zed_node/depth/depth_registered", Image, callback)
rospy.spin()
