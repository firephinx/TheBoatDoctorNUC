#!/usr/bin/env python

import rospy 
from object_detection.srv import *

import argparse


######### cmd args ########
parser = argparse.ArgumentParser(description='kinect_service paramers')
parser.add_argument('--type', type=int, help='actuator type')
######################################## 


def client(want):
	rospy.wait_for_service("kinect2/locations") 
	get_status_info=rospy.ServiceProxy("kinect2/locations",callKinect)
	print 2
	get_status_info(want)
	print 1
	#print resp1	


if __name__=="__main__":
	rospy.init_node('kinect_client', anonymous=False)
	args = parser.parse_args()
	client(args.type)
	

