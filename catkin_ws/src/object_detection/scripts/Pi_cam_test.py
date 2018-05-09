#!/usr/bin/env python

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from rgb_processing import pi_cam_process 
import time 


import params 

from object_detection.srv import *

################### image-processing (will move later) #############



## type 1: shuttle valvue   0: horizontal 1:vertical  
## type 2: orange vale      
## type 3: spigot valve     0: horizontal 1:vertical
## type 4: breaker 











#################### added ##########################

class image_converter:

  def __init__(self,type,subType):
    #self.out = cv2.VideoWriter('outpy.avi',cv2.VideoWriter_fourcc('X','V','I','D'), 10, (270,480))
    #fourcc = cv2.VideoWriter_fourcc(*'XVID')
    #self.out = cv2.VideoWriter('output.avi',fourcc, 20.0, (640,480))
    print "im in PI_cam_test"
    self.collect_data=0 
    self.imageNum=869 
    self.bridge = CvBridge()
    #self.image_sub = rospy.Subscriber("/kinect2/hd/image_colorcc",Image,self.RGB_proc)
    self.image_sub = rospy.Subscriber("/raspicam_node/image/raw",Image,self.RGB_proc)
    self.image_pub=rospy.Publisher("/raspicam_node/image/raw_rotated",Image,queue_size=10) ### rotate image horizontally
    self.status_pub=rospy.Publisher("/raspicam_node/actuator_status",String,queue_size=10) ### publish actuator 
    self.error_pub=rospy.Publisher("/raspicam_node/error_status",String,queue_size=10)
    #self.pub = rospy.Publisher('/kinect2/qhd/RGB_proc', Image, queue_size=10)
    #self.image_sub2 = rospy.Subscriber("/kinect2/sd/image_depth",Image,self.Depth_proc)
    self.type=type 
    self.subType=subType ## this is the sub type for an actuator
    self.dataNum=0 
    self.dataQ=params.dataQ_pi

    self.track=0 
    self.max_track=params.max_track_pi

    self.values=[] 

  def RGB_proc(self,RGB_stream):
    try:
      #data.encoding='mono16'
      print "im in RGB_proc"
      self.track+=1
      cv_image = self.bridge.imgmsg_to_cv2(RGB_stream, desired_encoding="bgr8")
      ####### after new mount, comment out the two lines below ####### 
      #cv_image=cv2.flip(cv_image,0)  ## flip image horizontally 
      #cv_image=cv2.flip(cv_image,1)  ## flip image vertically 
      #############################################################
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image,"bgr8"))
      # cv2.imshow("RGB",cv_image)
      # key=cv2.waitKey(20)

      PI=pi_cam_process(cv_image)
      value=PI.locate_green(self.type,cv_image,self.subType)  ## will be all in str type
      if value is not None: 
        self.dataNum+=1
        if self.type ==1 or self.type==2 or self.type==3:
          self.values.append(float(value))
          print float(value)
        elif self.type==4:
          results=[]  ### up is 1 
          for elements in value:
            result=elements=='up'
            results.append(result)
          self.values.append(results)
        else:
          print "[RGB_proc] (1) Type should be 1 to 4"
      else: 
        print "[RGB_proc] Doesn't find good things"
        status=String()
        status.data=str(10000)  ### publish error for raspiCAM
        #self.error_pub.publish(status)
        self.status_pub.publish(status)



      if self.track==self.max_track:
        print "Pi cam failed"
        self.image_sub.unregister()
        Final_value=String() 
        Final_value.data='failed'
        #self.status_pub.publish(Final_value)  ## publish angle 
        self.values=[]  ##  reset 
        print "shutdown a node"

      if self.dataNum==self.dataQ:  ## analyze dataQ number of images
          if self.type ==1 or self.type==2 or self.type==3:
            final_value=str(np.mean(self.values))     ### final type is a string 
          elif self.type==4:
            final_value=np.array(self.values)
            final_value=np.mean(final_value,axis=0)
            final_value=final_value//0.5  ### make it biase to up if mean is >= 0.5
            str1=str(final_value[0])
            str2=str(final_value[1])
            str3=str(final_value[2])
            final_value=str1+str2+str3
          else:
            print "[RGB_proc] (2) Type should be 1 to 4"

          print "unregister topic" 
          self.image_sub.unregister()
          print "Final value",final_value
          Final_value=String() 
          Final_value.data=final_value
          #time.sleep(0.01)
          self.status_pub.publish(Final_value)  ## publish angle 
          self.values=[]  ##  reset 
          print "shutdown a node"




      # if key==ord('e'):
      #   rospy.signal_shutdown("User exit")

      
    except CvBridgeError as e:
      print(e)

    # (rows,cols,channels) = cv_image.shape
    # if cols > 60 and rows > 60 :
    #   cv2.circle(cv_image, (50,50), 10, 255)  

    #rospy.loginfo(cv2.__version__)
    # try:
    #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    # except CvBridgeError as e:
    #   print(e)























# def main(args):
#   ic = image_converter()
#   try:
#     rospy.spin()
#   except KeyboardInterrupt:
#     print("Shutting down")
#   cv2.destroyAllWindows()

# if __name__ == '__main__':
#     main(sys.argv)


def get_actuator_status(msg):
  ic = image_converter(msg.Type,msg.subType)
  rospy.spin()

if __name__ == '__main__':
  rospy.init_node('raspi_location', anonymous=True)
  s=rospy.Service("raspicam_service/actuator_status",callKinect,get_actuator_status)
  rospy.spin()
