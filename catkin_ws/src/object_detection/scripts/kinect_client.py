#!/usr/bin/env python

import rospy 
from object_detection.srv import *

import argparse


######### cmd args ########
parser = argparse.ArgumentParser(description='kinect_service paramers')
parser.add_argument('--type', type=int, help='actuator type')
parser.add_argument('--subType',type=int,help='actuator subType')
######################################## 


def client(Type,subType):
	print "subType",subType
	rospy.wait_for_service("kinect2/locations") 
	get_status_info=rospy.ServiceProxy("kinect2/locations",callKinect)
	print 2
	get_status_info(Type,subType)
	print 1
	#print resp1	


if __name__=="__main__":
	rospy.init_node('kinect_client', anonymous=False)
	args = parser.parse_args()
	client(args.type,args.subType)
	

