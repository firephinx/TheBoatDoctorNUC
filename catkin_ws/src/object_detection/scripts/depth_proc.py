#!/usr/bin/env python

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image,CameraInfo
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import message_filters
from std_msgs.msg import Float32MultiArray



from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import struct
import ctypes
import time
import copy
import math 


from rgb_processing import kinect_process

from visualization_msgs.msg import Marker

import tf
import tf.msg
import geometry_msgs.msg


import params 

from object_detection.srv import *


## type 1: shuttle valvue 1 -1 (2): horizontal 1 (1): verical  
## type 2: orange vale 
## type 3: spigot valve 3 -1 (2): L shape   3 1 (1): small 
## type 4: breaker   up 1 (1), down -1 (2)



class image_converter:

  def __init__(self,type,stationF):
    #self.out = cv2.VideoWriter('outpy.avi',cv2.VideoWriter_fourcc('X','V','I','D'), 10, (270,480))
    #fourcc = cv2.VideoWriter_fourcc(*'XVID')
    #self.out = cv2.VideoWriter('output.avi',fourcc, 20.0, (640,480))
    self.collect_data=0 
    self.imageNum=869 
    self.bridge = CvBridge()
    self.alpha=0.5 ## blending factor for depth + color 
    self.gamma=0 ## blending shift .....
    self.scale=1 ### sacle img sizes
    self.th_xy=11 ### threshold to account for having object for measuring X,Y

    self.marker=Marker()

    self.max_track=params.max_track_kinect

    #self.image_sub = rospy.Subscriber("/kinect2/qhd/points",PointCloud2,self.depth_proc)

    self.depth_sub = message_filters.Subscriber("/kinect2/qhd/image_depth_rect",Image)
    self.color_sub=message_filters.Subscriber("/kinect2/qhd/image_color_rect",Image)
    self.cameraInfo_sub=message_filters.Subscriber("/kinect2/qhd/camera_info",CameraInfo)
    self.position_pub = rospy.Publisher('/Marker/actuator_pose', Marker, queue_size=10)
    self.pub_actuator_tf = rospy.Publisher("/tf/actuator", tf.msg.tfMessage,queue_size=10)  ###### try to use tf format, not using 
    self.pub_actuator_own=rospy.Publisher("/kinect2/actuator_location",Float32MultiArray,queue_size=10)
    self.pub_error=rospy.Publisher("/kinect2/error_msg",String,queue_size=10)
    ts=message_filters.TimeSynchronizer([self.depth_sub,self.color_sub,self.cameraInfo_sub],10)
    ts.registerCallback(self.get_data)
    self.depth_stream=None 
    self.color_stream=None 
    self.camera_info=None 


    self.br = tf.TransformBroadcaster()
    self.tf_listener=tf.TransformListener()


    self.dataNum=0

    self.type=type ### actuator type 

    self.X=[] ##### locations of acutators 
    self.Y=[]
    self.Z=[]
    self.SUBTYPE=[] ## subtype of some acutators 
    self.bStatus=[] ### breakerStatus
    self.dataQ=params.dataQ_kinect	### num of images to process, use odd number to break tie  
    self.station=stationF
    self.track=0 
  


  def pub_actuator_Marker(self,x,y,z):
		  	self.marker.header.frame_id = "/my_base_rgb_optical_frame"
			#self.robotMarker.header.stamp = rospy.get_rostime()
			self.marker.type = self.marker.CUBE
			self.marker.action = self.marker.ADD
			self.marker.scale.x = 0.05
			self.marker.scale.y = 0.05
			self.marker.scale.z = 0.01
			self.marker.color.a = 1.0
			self.marker.color.r = 1.0
			self.marker.color.g = 1.0
			self.marker.color.b = 0.0
			self.marker.pose.orientation.w = 1.0
			self.marker.pose.position.x = x/1000
			self.marker.pose.position.y = y/1000
			self.marker.pose.position.z = z/1000
			self.position_pub.publish(self.marker)


  def pub_actuator(self,x,y,z):
			t = geometry_msgs.msg.TransformStamped()
			t.header.frame_id = "/my_base_rgb_optical_frame"
			t.header.stamp = rospy.Time.now()
			t.child_frame_id = "actuator"
			t.transform.translation.x = x/1000
			t.transform.translation.y = y/1000
			t.transform.translation.z = z/1000

			t.transform.rotation.x = 0.0
			t.transform.rotation.y = 0.0
			t.transform.rotation.z = 0.0
			t.transform.rotation.w = 1.0

			tfm = tf.msg.tfMessage([t])
			self.pub_actuator_tf.publish(tfm)
  
  def broadcast_tf(self,x,y,z):
  			self.br.sendTransform((x/1000, y/1000, z/1000),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "/actuator",
                         "/my_base_rgb_optical_frame")
  			#print 1


  def get_data(self,depth_stream,color_stream,camera_info):
  		self.depth_stream=depth_stream
  		self.color_stream=color_stream
  		self.camera_info=camera_info
  		#print self.type
  		print "[Kinect] getting data"  		
  		x,y,z,subType,error,breakerStatus=self.img_proc()
  		#print 11111111111111111111111111111111111111, x,y,z,subType,error,breakerStatus 
  		self.track+=1
  		if error==0: ## if no error 
  			self.dataNum+=1   			
	  		if subType is None: ### convert subType to ROSmsg compatible data type 
	  			subType=0

	  		if self.type==41 or self.type==42:
	  			self.SUBTYPE.append(subType)	  		
	  			self.X.append(x) ## first breaker 
	  			self.Y.append(y) ## second breaker 
	  			self.Z.append(z) ## third breaker
	  			self.bStatus.append(breakerStatus)
	  				
	  		else:
		  		self.SUBTYPE.append(subType)	  		
		  		self.X.append(x)
		  		self.Y.append(y)
		  		self.Z.append(z)
		  		self.bStatus.append([0,0,0]) ### trivial since not used 
	  		#print x,y,z
	  	else:
	  		message=String() 
	  		message.data=str(error)
	  		self.pub_error.publish(message)  ### publish error message, adjust robot position 	


	  	if self.track==self.max_track:
	  		print "Kinect failed"
  			self.depth_sub.unregister()
  			self.color_sub.unregister()
  			self.cameraInfo_sub.unregister()
  			location=Float32MultiArray()
	  		location.data=[10000,10000]
  			# self.pub_actuator_own.publish(location)
  			self.X=[]  ## reset 
  			self.Y=[]
  			self.Z=[]
  			self.SUBTYPE=[]
  			self.bStatus=[]

  		if  self.dataNum==self.dataQ:
  			print "unsubscrib topic"
  			self.depth_sub.unregister()
  			self.color_sub.unregister()
  			self.cameraInfo_sub.unregister()
  			if self.type==41 or self.type==42:
  				pt1=np.mean(np.array(self.X),axis=0)
  				pt2=np.mean(np.array(self.Y),axis=0)
  				pt3=np.mean(np.array(self.Z),axis=0)
  				subType=0
  				status=np.sum(np.array(self.bStatus),axis=0)
  				#print self.bStatus
  				#print status
  				for i in np.arange(3):
	  				if status[i]>0.5:
		  				status[i]=1 
		  			elif status[i]<-0.5: 
		  				status[i]=2 
		  			else:
		  				#print "im here"
		  				status[i]=0
		  		location=Float32MultiArray()
	  			location.data=[pt1[0],pt1[1],pt1[2],pt2[0],pt2[1],pt2[2],pt3[0],pt3[1],pt3[2],subType,status[0],status[1],status[2]]
  			else:
	  			xVal=np.mean(self.X)
	  			yVal=np.mean(self.Y)
	  			zVal=np.mean(self.Z)
	  			subType=np.sum(self.SUBTYPE)
	  			if subType>0.5:
	  				subType=1 
	  			elif subType<-0.5: 
	  				subType=2 
	  			else:
	  				subType=0
	  			print "The x,y,z values are",xVal,yVal,zVal
	  			location=Float32MultiArray()
	  			location.data=[xVal,yVal,zVal,subType]
  			self.pub_actuator_own.publish(location)
  			self.X=[]  ## reset 
  			self.Y=[]
  			self.Z=[]
  			self.SUBTYPE=[]
  			self.bStatus=[]

  			#cv2.destroyAllWindows()
  		
  				

  def img_proc(self):
  	### This function process depth and RGB images to get x,y,z info of an interested obj 
  		#print (4)
  		error=0
  		print "in img_proc"
  		if self.depth_stream is not None and self.color_stream is not None and self.camera_info is not None:
			self.depth_stream.encoding='mono16'
			cv_depth=self.bridge.imgmsg_to_cv2(self.depth_stream,desired_encoding='mono16')  ## distance (mm) coded in UC161 (or mono16)
			cv_depth=cv2.resize(cv_depth,(0,0,),fx=self.scale,fy=self.scale)

			#print (2)
			cv_depth_visual=copy.deepcopy(cv_depth)
			cv2.normalize(cv_depth_visual, cv_depth_visual, 0, 65535, cv2.NORM_MINMAX)  ## uncomment for better visual result 

			# cv2.imshow("depth_img",cv_depth_visual)
			# cv2.waitKey(20)
			#print np.max(cv_depth)
			#print (3)

			cv_color=self.bridge.imgmsg_to_cv2(self.color_stream,desired_encoding='bgr8')
			cv_color=cv2.resize(cv_color,(0,0,),fx=self.scale,fy=self.scale)
			# cv2.imshow("color_img",cv_color)
			# cv2.waitKey(20)

			#### visually check if depth img and color img aligned, will comment out ####
			#depth_mask=np.dstack((cv_depth,cv_depth,cv_depth))
			#depth_color=cv2.addWeighted(depth_mask,self.alpha,cv_color,1-self.alpha,self.gamma)
			#cv2.imshow("color_depth",depth_color)
			#cv2.waitKey(1)
			#############################Visual check end ####################################

			#### find actuator ####
			kinect=kinect_process(cv_color)
			target_masks,subType,breakerStatus=kinect.locate_actuators(self.type,self.station,cv_color) 
			#print 222222222222222,target_masks,subType,breakerStatus
			if target_masks is not None:
				if self.type==41 or self.type==42:
					xyz=[]
					for target_mask in target_masks:
						mask_final=np.uint8(target_mask)
						#### end #### 

						#### depth result #####
						depth=self.Z_proc(cv_color,cv_depth,mask_final)

						X,Y=self.XY_proc(cv_color,mask_final,self.camera_info,cv_depth,self.th_xy)
						print "X:{} mm  Y:{} mm".format(X,Y)
						self.pub_actuator_Marker(-Y,X,depth) ### switch XY to comply with Kinect rgb optical frame 
						#self.pub_actuator(X,Y,depth)
						self.broadcast_tf(-Y,X,depth)
						rospy.sleep(0.01)
						(trans,rot)=self.tf_listener.lookupTransform('/IK_reference','/actuator',rospy.Time(0))
						xyz.append([trans[0],trans[1],trans[2]])
						#return X,Y,depth
					if breakerStatus!=None:  ### has to find breakerStatus for type 4 
						error=0
					else:
						error=1 
					return xyz[0],xyz[1],xyz[2],subType,error,breakerStatus

				else:
					mask_final=np.uint8(target_masks)
					#### end #### 

					#### depth result #####
					depth=self.Z_proc(cv_color,cv_depth,mask_final)

					X,Y=self.XY_proc(cv_color,mask_final,self.camera_info,cv_depth,self.th_xy)
					print "X:{} mm  Y:{} mm".format(X,Y)
					self.pub_actuator_Marker(-Y,X,depth) ### switch XY to comply with Kinect rgb optical frame 
					#self.pub_actuator(X,Y,depth)
					self.broadcast_tf(-Y,X,depth)
					rospy.sleep(0.01)
					try:
						(trans,rot)=self.tf_listener.lookupTransform('/IK_reference','/actuator',rospy.Time(0))
						print trans[0],trans[1],trans[2]
						#return X,Y,depth
						if self.type ==  2:
							error=0 
							return trans[0],trans[1],trans[2],subType,error,None
						else:
							if subType is None: 
								error=1
								return trans[0],trans[1],trans[2],subType,error,None  
							else: 
								error=0
								return trans[0],trans[1],trans[2],subType,error,None  ## none is breakerstatus
					except:
						print "[depth_proc/img_proc] I can't find IK_reference" 
						error=1
						return -1000000,-100000,-100000,None,error,None 
			
			else:
				print "[depth_proc/img_proc] I can't find actuator" 
				error=1
				return -1000000,-100000,-100000,None,error,None 
	


			self.depth_stream=None 
			self.color_stream=None 
			self.camera_info=None
				
		else:
			print "[depth_proc/img_proc] no data"
			error=1 
			return -1000000,-100000,-100000,None,error 



  def Z_proc(self,cv_color,cv_depth,mask_final):


	  
		#### helper line ####
		#mask_final=np.uint8(target_mask)
		#masked_img=cv2.bitwise_and(cv_color,cv_color,mask=mask_final)
		#cv2.imshow("result4",masked_img)
		#cv2.waitKey(200)
		### ends ####

		#### helper line (check if correct mask) ####	
		### ends ####
		
		masked_depth=cv2.bitwise_and(cv_depth,cv_depth,mask=mask_final)  ## filtered depth map
		depth=np.true_divide(np.sum(masked_depth),np.sum(masked_depth!=0)) ### final depth of target, by avareging distances of all possible pixels 
		#print np.sum(masked_depth)
		#print np.sum(masked_depth!=0)
		if math.isnan(depth):
			print "[depth_proc] No object found"
			depth=0 ### not a valid number 
		else:
			print "depth:",depth,"mm"
		return depth 
  
  def XY_proc(self,cv_color,mask_final,camera_info,cv_depth,th_xy):
		

		### helper line ########
		masked_color=cv2.bitwise_and(cv_color,cv_color,mask=mask_final)
		# cv2.imshow("result4",masked_color)
		# cv2.waitKey(20)
		### ends ########## 

		points=cv2.findNonZero(mask_final)
		if points is None:
			print "[depth_proc.py] No points found"
			X=0
			Y=0
			return X,Y
		else:
			if len(points)>th_xy:
				num_pts=np.shape(points)[0]
				#print np.shape(points)
				points=np.reshape(points,(np.shape(points)[0],np.shape(points)[2])) ## n by 2
				#print points[0,:] 
				new_coln=np.roll(points,1,axis=1) ### flip colns for np.array use: row, coln instead of x,y(openCV style)
				#print new_coln
				#print np.max(points)
				#print np.shape(cv_color)
				#print np.shape(cv_depth)
				#print points[0]
				#print [reversed(tuple(i)) for i in points]
				#print points
				depths= [cv_depth[tuple(i)]for i in new_coln]
				depths=np .reshape(np.array(depths),(1,len(depths))) ### 1 by n 
				#print np.shape(depths)

				Ones=np.ones([num_pts,1]) ### n by 1

				uv_homo=np.concatenate((points,Ones),axis=1) ### n by 3
				uv_homo=uv_homo.T  ### 3 by n
				projection=np.reshape(camera_info.P,(3,4))	### 3 by 4
				XY_homo=np.dot(np.linalg.pinv(projection),uv_homo) ### 4 by n
				#print XY_homo
				XY=depths/XY_homo[2,:]*XY_homo[0:2,:]
				#print XY[:,0]
				#print XY_homo[:,0]
				XY_avg=np.average(XY,axis=1)
				#print np.dot(np.linalg.pinv(projection),uv_homo[:,10000])
				X=XY_avg[0]/self.scale
				Y=XY_avg[1]/self.scale
				return X,Y
			else:
				print "[depth_proc.py] No points found"
				X=0
				Y=0
				return X,Y
			#print np.linalg.pinv(projection)

			### depth match ###


			#XY_homo=np.dot(ny.linalg.pinv(camera_info.P),avg_uv)









  	



  
 #  def Depth_proc(self,depth_stream):
	# try:
	#         #depth_stream.encoding='16UC1'
	#         depth_stream.encoding='mono16'
	#         cv_img = self.bridge.imgmsg_to_cv2(depth_stream, desired_encoding="mono16")
	#         print np.max(cv_img)
	       
	#         #print depth_stream
	#         #print cv_depth
	        

	# except CvBridgeError as e:
	#         print(e)


 #  def depth_proc(self,ptCld):
 #  	try:
	#         #data.encoding='mono16'
	#         #depth_stream.encoding='mono16'
	#         #cv_depth = self.bridge.imgmsg_to_cv2(depth_stream, desired_encoding="mono16")
	#         #print depth_stream
	#         #print cv_depth
	#         #cv_depth=cv2.resize(cv_depth,(0,0,),fx=1.5,fy=1.5)
	#         #cv2.normalize(cv_depth, cv_depth, 0, 65535, cv2.NORM_MINMAX)
	#         #print cv_depth
	       
	#         #cv2.imshow('depth',cv_depth)
	#         #cv2.waitKey(20)

	#         gen=pc2.read_points(ptCld,skip_nans=True)
	#         int_data=list(gen)
	#         #print len(int_data)
	#         R=[]
	#         G=[]
	#         B=[]
	#         j=0
	#         for x in int_data:
	# 	        test = x[3] 
	# 	        # cast float32 to int so that bitwise operations are possible
	# 	        s = struct.pack('>f' ,test)
	# 	        #print struct.unpack('>l',s)
	# 	        i = struct.unpack('>l',s)[0]
	# 	        # you can get back the float value by the inverse operations
	# 	        pack = ctypes.c_uint32(i).value
	# 	        #print pack
	# 	        r = (pack & 0x00FF0000)>> 16
	# 	        g = (pack & 0x0000FF00)>> 8
	# 	        b = (pack & 0x000000FF)
	# 	        j=j+1
	# 	        #print j
	# 	        #print r,g,b
	# 	        R.append(r)
	# 	        G.append(g)
	# 	        B.append(b)
	        
	#         print len(R)



	# except CvBridgeError as e:
	#         print(e)










def localize_actuators(msg):
	ic = image_converter(msg.Type,msg.subType)
	rospy.spin() 
	print "im here"

if __name__ == '__main__':
	rospy.init_node('kinect2_FindLocations', anonymous=True)
  	s=rospy.Service("kinect2/locations",callKinect,localize_actuators)
  	rospy.spin()

