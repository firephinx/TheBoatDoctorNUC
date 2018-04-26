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
	
	def __init__(self,Type):
		#rospy.init_node('CV_node', anonymous=False)

		self.data=None
		self.firstCall=1
		self.firstCall_pi=1
		#self.type=Type
		if(Type == 'V1'):
			self.type = 3
		elif(Type == 'V2'):
			self.type = 2
		elif(Type == 'V3'):
			self.type = 1
		elif(Type == 'A'):
			self.type = 4
		elif(Type == 'B'):
			self.type = 4
		else:
			self.type = 0

	def get_station_info_kinect(self):
		Type=str(self.type)
		command="gnome-terminal -e 'rosrun object_detection kinect_client.py --type {0}'".format(Type)
		if self.firstCall==1:
			#os.system("gnome-terminal -e 'rosrun object_detection pi_cam_client.py --type 3'") ## call rosservice in a new terminal
			os.system(command) ## call rosservice in a new terminal 
			time.sleep(0.01) ## give time for os to start 
			self.firstCall=0
			print 3
			msg = rospy.wait_for_message('/kinect2/actuator_location', Float32MultiArray) ## message received from publish topic in kinect 
 			#msg = rospy.wait_for_message('test',Bool)
 			print 2
 			self.data=msg.data
 			command="/home/theboatdoctor-nuc/TheBoatDoctorNUC/catkin_ws/src/object_detection/scripts/kinect_client.py --type {0}".format(Type)
 			pid=check_output(["pidof","python",command]) 
 			print pid 
 			#time.sleep(1000000) ## glook for pid			
 			os.kill(int(pid.split(" ")[0]), signal.SIGKILL)

 			if len(self.data)==1:
 				return 10000  ### error msg 
 			else:
 				if(self.type == 4):
 					station_object_position_in_3d = self.data[0:9]
 					station_orientation = ['','','']
 					for i in xrange(3):
 						if(self.data[i+9] == 1):
 							station_orientation[i] = 'up'
 						else:
 							station_orientation[i] = 'down'
 					return (station_object_position_in_3d, station_orientation)
 				elif(self.type == 2):
 					station_object_position_in_3d = self.data[0:3]
 					station_orientation = 'vertical'
 				else:
 					station_object_position_in_3d = self.data[0:3]
 					if(self.data[3] == 1):
 						station_orientation = 'vertical'
 					elif(self.data[3] == 2): 
 						station_orientation = 'horizontal'
 					else:
 						station_orientation = ''
 				return (station_object_position_in_3d, station_orientation)
 			
		else: 
 			msg = rospy.wait_for_message('/kinect2/actuator_location', Float32MultiArray) ## message received from publish topic in kinect 
 			self.data=msg.data
 			if len(self.data)==1:
 				return 10000  ### error msg 
 			else:
 				return self.data

 	def get_station_info_pi(self):
 		Type=str(self.type)
 		#print 1
		command="gnome-terminal -e 'rosrun object_detection pi_cam_client.py --type {0}'".format(Type)
		#print command
		#command="xterm -hold -e 'source /home/theboatdoctor-nuc/.bashrc && rosservice call /raspicam_service/actuator_status {0}'".format(Type)		
		#print 2
		if self.firstCall_pi==1:
			print "im here"
			os.system(command) ## call rosservice in a new terminal 
			print "im here 2"
			time.sleep(0.01)
			self.firstCall_pi=0
			print 3
			msg = rospy.wait_for_message('/raspicam_node/actuator_status',String) ## message received from publish topic in raspcam
 			self.data=msg.data
 			print self.data
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
	detector=TheBoatDoctorCV(1)
	location=detector.get_station_info_pi()
	#location=detector.get_station_info_kinect()
	print location
	print "finished"
