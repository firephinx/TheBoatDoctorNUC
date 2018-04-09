#!/usr/bin/env python

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from rgb_processing import pi_cam_process 


################### image-processing (will move later) #############















#################### added ##########################

class image_converter:

  def __init__(self):
    #self.out = cv2.VideoWriter('outpy.avi',cv2.VideoWriter_fourcc('X','V','I','D'), 10, (270,480))
    #fourcc = cv2.VideoWriter_fourcc(*'XVID')
    #self.out = cv2.VideoWriter('output.avi',fourcc, 20.0, (640,480))
    self.collect_data=0 
    self.imageNum=869 
    self.bridge = CvBridge()
    #self.image_sub = rospy.Subscriber("/kinect2/hd/image_color",Image,self.RGB_proc)
    self.image_sub = rospy.Subscriber("/raspicam_node/image/raw",Image,self.RGB_proc)
    self.image_pub=rospy.Publisher("/raspicam_node/image/raw_rotated",Image,queue_size=10) ### rotate image horizontally
    #self.pub = rospy.Publisher('/kinect2/qhd/RGB_proc', Image, queue_size=10)
    #self.image_sub2 = rospy.Subscriber("/kinect2/sd/image_depth",Image,self.Depth_proc)
  
  def RGB_proc(self,RGB_stream):
    try:
      #data.encoding='mono16'

      cv_image = self.bridge.imgmsg_to_cv2(RGB_stream, desired_encoding="bgr8")
      cv_image=cv2.flip(cv_image,0)  ## flip image horizontally 
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image,"bgr8"))
      #cv2.imshow("RGB",cv_image)
      #key=cv2.waitKey(0)

      PI=pi_cam_process(cv_image)
      Type=3
      final_mask=PI.locate_green(Type,cv_image)


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























def main(args):
  rospy.init_node('raspicam_cam', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)