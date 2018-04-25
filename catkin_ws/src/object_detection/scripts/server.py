#!/usr/bin/env python
# import sys
#sys.path.insert(0,"/home/theboatdoctor-nuc/TheBoatDoctorNUC/catkin_ws/src/object_detection/srv")

from object_detection.srv import *
#import callKinect.srv 
import rospy

def wantToResponse(req):
	print req.want 
	x=1
	y=2
	z=3
	return x,y,z 

def add_two_ints_server():
	rospy.init_node("try")
	s=rospy.Service("call_it_service",callKinect,wantToResponse)
	print "good" 	
	rospy.spin()

if __name__=="__main__":
	add_two_ints_server()  
