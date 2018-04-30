#!/usr/bin/env python

import rospy 
from object_detection.srv import *

import argparse


######### cmd args ########
parser = argparse.ArgumentParser(description='raspicam_service paramers')
parser.add_argument('--type', type=int, help='actuator type')
parser.add_argument('--subType',type=int,help='actuator subType')
######################################## 


def client(Type,subType):
	#print want
	#print 1
	rospy.wait_for_service("/raspicam_service/actuator_status") 
	#print 2
	get_status_info=rospy.ServiceProxy("/raspicam_service/actuator_status",callKinect)
	print "inside"
	get_status_info(Type,subType)
	print "inside2"
	#print resp1	


if __name__=="__main__":
	rospy.init_node('pi_cam_client', anonymous=False)
	args = parser.parse_args()
	client(args.type,args.subType)
	
