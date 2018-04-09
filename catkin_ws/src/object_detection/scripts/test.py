#!/usr/bin/env python

import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from tracking import ActuatorClassifier as ACtracker 

class image_converter:

  def __init__(self):
    #self.out = cv2.VideoWriter('outpy.avi',cv2.VideoWriter_fourcc('X','V','I','D'), 10, (270,480))
    #fourcc = cv2.VideoWriter_fourcc(*'XVID')
    #self.out = cv2.VideoWriter('output.avi',fourcc, 20.0, (640,480)) 
    self.seeImage=0
    self.bridge = CvBridge()
    self.tracker=ACtracker()
    self.image_sub = rospy.Subscriber("/kinect2/hd/image_color",Image,self.callback)
  def callback(self,data):
    try:
      #data.encoding='mono16'

      cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
      #print np.shape(cv_image)
      #self.out.write(cv_image)
      #cv_iamge=cv2.resize(cv_image,(0,0,),fx=0.6,fy=0.6)
      #print np.shape(cv_image)
      ######  write image ##########
      #filePath="/home/foredawnlin/test_ROS/src/recognize/images/"
      #fileName=str(self.imageNum)+".jpg"
      #path=filePath+fileName
      #print (path)
      #cv2.imwrite(path,cv_image)
      #print ("im here")
      #self.imageNum+=1
      ###### write image done ########


      a,b,c,d=self.tracker.get_classification(cv_image)
      #print a,b,c,d
      #print "box",a 
      print "scores",b 
      print "classes",c 
      #print "num",d 

      height,width,_=np.shape(cv_image)
      #print width,height
      detected_counts=len([scores for scores in b[0] if scores>=0.01]) ## was 0.01

      ##### draw detection #####
      
      font = cv2.FONT_HERSHEY_SIMPLEX
      classes=[] 
      for i in np.arange(0,detected_counts):
          class_name=str(c[0][i])
          if class_name not in classes:
            classes.append(class_name)
            y=int(a[0][i][0]*height)
            x=int(a[0][i][1]*width)
            h=int((a[0][i][2]-a[0][i][0])*height)
            w=int((a[0][i][3]-a[0][i][1])*width)
            img = cv2.rectangle(cv_image,(x,y),(x+w,y+h),(0,255,0),2)          
            cv2.putText(cv_image,class_name,(x,y),font,0.8,(255,0,0),2,cv2.LINE_AA)
      print classes

      ##### draw detection done ########

      cv2.imshow('see',cv_image)
      key=cv2.waitKey(25)
      if key==ord('q'):
        cv2.destroyAllWindows()
        rospy.signal_shutdown("User exit")

    except CvBridgeError as e:
      print(e)

    # (rows,cols,channels) = cv_image.shape
    # if cols > 60 and rows > 60 :
    #   cv2.circle(cv_image, (50,50), 10, 255)  

    #cv2.imshow("Image window", cv_resized)
    #key=cv2.waitKey(3)
    #rospy.loginfo(cv2.__version__)
    # try:
    #   self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    # except CvBridgeError as e:
    #   print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)