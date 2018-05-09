#!/usr/bin/env python 

import rospy
import os
import time 



## import ros msg
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String

## to kill terminal inside python
from subprocess import check_output
import signal




class TheBoatDoctorCV():
	
	def __init__(self,Type,subType):
		rospy.init_node('test', anonymous=False)
		self.type=Type
		self.data=None
		self.firstCall=1 
		self.subType=subType

	def get_actuator_location(self):
		Type=str(self.type)
		subType=str(self.subType)
		command="gnome-terminal -e 'rosrun object_detection kinect_client.py --type {0} --subType {1}'".format(Type,subType)
		if self.firstCall==1:
			#os.system("gnome-terminal -e 'rosrun object_detection pi_cam_client.py --type 3'") ## call rosservice in a new terminal
			os.system(command) ## call rosservice in a new terminal 
			time.sleep(0.01) ## give time for os to start 
			self.firstCall=0
			msg = rospy.wait_for_message('/kinect2/actuator_location', Float32MultiArray) ## message received from publish topic in kinect 
 			self.data=msg.data
 			command="/home/theboatdoctor-nuc/TheBoatDoctorNUC/catkin_ws/src/object_detection/scripts/kinect_client.py --type {0}".format(Type)
 			pid=check_output(["pidof","python",command]) 
 			print pid 
 			#time.sleep(1000000) ## glook for pid			
 			os.kill(int(pid.split(" ")[0]), signal.SIGKILL)

 			if len(self.data)==1:
 				return 10000  ### error msg 
 			else:
 				return self.data
 			
		else: 
 			msg = rospy.wait_for_message('/kinect2/actuator_location', Float32MultiArray) ## message received from publish topic in kinect 
 			self.data=msg.data
 			if len(self.data)==1:
 				return 10000  ### error msg 
 			else:
 				return self.data


 	def get_actuator_status(self):
 		Type=str(self.type)
 		subType=str(self.subType)
		command="gnome-terminal -e 'rosrun object_detection pi_cam_client.py --type {0} --subType {1}'".format(Type,subType)
		#print command
		#command="xterm -hold -e 'source /home/theboatdoctor-nuc/.bashrc && rosservice call /raspicam_service/actuator_status {0}'".format(Type)		
		if self.firstCall==1:
			time.sleep(0.01)
			os.system(command) ## call rosservice in a new terminal 
			time.sleep(0.01)
			self.firstCall=0
			msg = rospy.wait_for_message('/raspicam_node/actuator_status',String) ## message received from publish topic in kinect 
 			self.data=msg.data
 			command="/home/theboatdoctor-nuc/TheBoatDoctorNUC/catkin_ws/src/object_detection/scripts/pi_cam_client.py --type {0}".format(Type)
 			pid=check_output(["pidof","python",command]) 
 			os.kill(int(pid.split(" ")[0]), signal.SIGKILL)

 			if len(self.data)==1:
 				return 10000  ### error msg 
 			else:
 				return self.data 			

		else: 
 			msg = rospy.wait_for_message('/raspicam_node/actuator_status', String) ## message received from publish topic in kinect 
 			self.data=msg.data
 			if len(self.data)==1:
 				return 10000  ### error msg 
 			else:
 				return self.data


if __name__=="__main__":
	detector=TheBoatDoctorCV(41,0)
	location=detector.get_actuator_location()
	#location=detector.get_actuator_status()
	
	print location
	print "finished"





