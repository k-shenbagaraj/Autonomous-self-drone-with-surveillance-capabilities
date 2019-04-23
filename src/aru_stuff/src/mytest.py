#!/usr/bin/env python

import numpy as np
import roslib
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import subprocess
import time
import cv2.aruco as aruco
x=0
var1=1
var2=1
var3=1
var4=1
cnt=0
cnt2=0
exit1=0
exit2=0
exit3=0
exit4=0
counter = 0

bridge = CvBridge()
def bebop_align(ids):
  while np.all(ids!=2):
    move(.1,0,0,0,0,0)
    while np.all(ids==2):
      break

def cam_move (cam_y,cam_z):
  
  #time.sleep(3)
  cam_movement = rospy.Publisher("/bebop/camera_control", Twist, queue_size= 10 )
  twist=Twist() 
  rate = rospy.Rate(10) 
  twist.linear.x = 0
  twist.linear.y = 0
  twist.linear.z = 0
  twist.angular.x = 0
  twist.angular.y= cam_y
  twist.angular.z= cam_z
  cam_movement.publish(twist)
   
    
def takeoff_drone ():
  
  p = subprocess.Popen(["rostopic","pub", "--once", "/bebop/takeoff", "std_msgs/Empty"],stdout=subprocess.PIPE)
  output,err = p.communicate()
  print output
  

def move (lx,ly,lz,ax,ay,az):
  
  movement = rospy.Publisher("/bebop/cmd_vel", Twist, queue_size = 10)
  rate_mov = rospy.Rate(10)
  movedrone = Twist()
  movedrone.linear.x = lx
  movedrone.linear.y = ly
  movedrone.linear.z = lz
  movedrone.angular.x = ax
  movedrone.angular.y = ay
  movedrone.angular.z = az
  movement.publish(movedrone)
  
    

def land_drone ():
  
  q = subprocess.Popen(["rostopic","pub", "--once", "/bebop/land", "std_msgs/Empty"],stdout=subprocess.PIPE)
  output1,err1 = q.communicate()
  print output1

def position(odom_data):
  global counter
  rospy.sleep(1)
  curr_time = odom_data.header.stamp
  pose = odom_data.pose.pose
  print(pose)

def image_callback(ros_image):
  global bridge
  global var1 
  global cnt
  global cnt2
  global var2
  global var3
  global var4
  global exit1
  global exit2
  global exit3
  global exit4

  
  #convert ros_image into an opencv-compatible image
  try:
    cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
  except CvBridgeError as e:
      print(e)
  
  gray1 = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
  aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
  parameters =  aruco.DetectorParameters_create()
  font = cv2.FONT_HERSHEY_SIMPLEX 
  corners, ids, rejectedImgPoints = aruco.detectMarkers(gray1, aruco_dict, parameters=parameters)
  gray = aruco.drawDetectedMarkers(cv_image, corners,ids,[0,0,255])
  global x
  
  if np.all(ids != None):
      x=x+1
      print(x)
      if np.all(ids==1):
        exit1=1
        

      if exit1:
        if np.all(ids == 1) and var1:

          
          gray = aruco.drawDetectedMarkers(cv_image, corners,ids,[0,255,255])
          cor2 = corners[0][0][0][1]
          cor1 = corners[0][0][0][0]
          id1a = corners[0][0][0][1]-corners[0][0][1][1]
          id1b = corners[0][0][0][0]-corners[0][0][1][0] 
          print id1a
          print id1b
                 
          if cor1>= 530 and var1:
            print("Move right",var1)
            move(0,-.05,0,0,0,0) 
          elif cor1<= 400 and var1:
            print("Move left",var1)
            move (0,.05,0,0,0,0)
          elif cor2>= 310 and var1:
            print("Move Back",var1)
            move (-0.05,0,0,0,0,0)
          elif cor2<= 190 and var1:
            print("Move Forward",var1)
            move (.05,0,0,0,0,0)
          elif 15>=id1a and id1b<=-2 and var1:
            print("Rotate clockwise",var1)
            move(0,0,0,0,0,-.1)
          elif 15>=id1a and id1b>=10 and var1:
            print("Rotate clockwise",var1)
            move(0,0,0,0,0,-.1)
          elif 25<=id1a and id1b<=-2 and var1:
            print("Rotate anticlockwise",var1)
            move(0,0,0,0,0,.1)
          elif 25<=id1a and id1b>=10 and var1:
            print("Rotate anticlockwise",var1)
            move(0,0,0,0,0,.1)
          elif 15<=id1a and id1a<=25 and id1b >=-2 and id1b <= 10 and cor1>= 400 and cor1 <= 530 and cor2>=190 and cor2<=310 and var1:
            var1=0
            print("Entered if loop",ids,cnt,var1)
            #move(.2,0,0,0,0,0)
          else:
            move(0,0,0,0,0,-0.1)
            print("Auto")
          cv2.putText(cv_image, "POINT 1", tuple(corners[0][0][0]), font, 1, (0,255,0),2,cv2.LINE_AA)
            
            
        elif var1==0 and exit1==1:
          
              cnt=cnt+1
              gray = aruco.drawDetectedMarkers(cv_image, corners,ids,[0,255,255])
              move(.08,-0.05,0,0,0,0)
              
              print("Searching for Point 2 - Move forward",ids,cnt)
              if np.all(ids==2):
                gray = aruco.drawDetectedMarkers(cv_image, corners,ids,[0,255,255])
                print("Reached Point 2")
                move(0,0,0,0,0,0)                
                exit1=0

      if np.all(ids==2):
        exit2=1
        

      if exit2:
        if np.all(ids == 2) and var2:

          
          gray = aruco.drawDetectedMarkers(cv_image, corners,ids,[0,255,255])
          cor2 = corners[0][0][0][1]
          cor1 = corners[0][0][0][0]
          id1a = corners[0][0][0][1]-corners[0][0][1][1]
          id1b = corners[0][0][0][0]-corners[0][0][1][0] 
          
          if cor2>= 310 and var2:
            print("Move Back")
            move (-0.05,0,0,0,0,0)
          elif cor2<= 190 and var2:
            print("Move Forward")    
            move (0.05,0,0,0,0,0)   
          elif cor1>= 530 and var2:
            print("Move right")
            move(0,-.05,0,0,0,0) 
          elif cor1<= 400 and var2:
            print("Move left")
            move (0,.05,0,0,0,0)
          elif cor2>= 310 and var2:
            print("Move Back")
            move (-0.05,0,0,0,0,0)
          elif cor2<= 190 and var2:
            print("Move Forward")
            move (.05,0,0,0,0,0)
          elif 15>=id1a and id1b<=-2 and var2:
            print("Rotate anticlockwise")
            move(0,0,0,0,0,-.1)
          elif 15>=id1a and id1b<=10 and var2:
            print("Rotate anticlockwise")
            move(0,0,0,0,0,-.1)
          elif 25<=id1a and id1b>=-2 and var2:
            print("Rotate clockwise")
            move(0,0,0,0,0,.1)
          elif 25<=id1a and id1b>=10 and var2:
            print("Rotate clockwise")
            move(0,0,0,0,0,.1)
          elif 15<=id1a and id1a<=25 and id1b >=-2 and id1b <= 10 and cor1>= 400 and cor1 <= 530 and cor2>=190 and cor2<=310 and var2:
            var2=0
            print("Entered if loop",ids,cnt)
          else:
            move(0,0,0,0,0,-0.1)
            print("Auto")
          cv2.putText(cv_image, "POINT 2", tuple(corners[0][0][0]), font, 1, (0,255,0),2,cv2.LINE_AA)
            
            
        elif var2==0 and exit2==1:
          
              cnt2=cnt2+1
              gray = aruco.drawDetectedMarkers(cv_image, corners,ids,[0,255,255])
              move(.05,-0.009,0,0,0,0)
              print("Searching for Point 3 - Move forward",ids,cnt2)
              if np.all(ids==3):
                gray = aruco.drawDetectedMarkers(cv_image, corners,ids,[0,255,255])
                print("Reached Point 3")
                move(0,0,0,0,0,0)                
                print("Exitted point 2 at point 3",ids)
                exit2=0
              

      if np.all(ids==3):
        exit3=1
        

      if exit3:
        if np.all(ids == 3) and var3:

          
          gray = aruco.drawDetectedMarkers(cv_image, corners,ids,[0,255,255])
          cor2 = corners[0][0][0][1]
          cor1 = corners[0][0][0][0]
          id1a = corners[0][0][0][1]-corners[0][0][1][1]
          id1b = corners[0][0][0][0]-corners[0][0][1][0] 
          
          if cor2>= 310 and var3:
            print("Move Back")
            move (-0.05,0,0,0,0,0)
          elif cor2<= 190 and var3:
            print("Move Forward") 
            move (0.05,0,0,0,0,0)      
          elif cor1>= 530 and var3:
            print("Move right")
            move(0,-.05,0,0,0,0) 
          elif cor1<= 400 and var3:
            print("Move left")
            move (0,.05,0,0,0,0)
          elif 15>=id1a and id1b<=-2 and var3:
            print("Rotate clockwise")
            move(0,0,0,0,0,-.1)
          elif 15>=id1a and id1b<=10 and var3:
            print("Rotate clockwise")
            move(0,0,0,0,0,-.1)
          elif 25<=id1a and id1b<=-2 and var3:
            print("Rotate anticlockwise")
            move(0,0,0,0,0,.1)
          elif 25<=id1a and id1b>=10 and var3:
            print("Rotate anticlockwise")
            move(0,0,0,0,0,.1)
          elif 15<=id1a and id1a<=25 and id1b >=-2 and id1b <=10 and cor1>= 400 and cor1 <= 530 and cor2>=190 and cor2<=310 and var3:
            var3=0
            print("Entered if loop",ids)
          else:
            move(0,0,0,0,0,-0.1)
            print("Auto")
          cv2.putText(cv_image, "POINT 3", tuple(corners[0][0][0]), font, 1, (0,255,0),2,cv2.LINE_AA)
            
            
        elif var3==0 and exit3==1:
          
              cnt=cnt+1
              gray = aruco.drawDetectedMarkers(cv_image, corners,ids,[0,255,255])
              move(.15,-0.09,0,0,0,0)
              print("Searching for Point 4 - Move forward",ids,cnt)
              if np.all(ids==4):
                gray = aruco.drawDetectedMarkers(cv_image, corners,ids,[0,255,255])
                print("Reached Point 4")
                move(0,0,0,0,0,0)                
                exit3=0

      if np.all(ids ==4):
        exit4=1
        

      if exit4:

        if np.all(ids == 4) and var4:
          gray = aruco.drawDetectedMarkers(cv_image, corners,ids,[0,255,255])
          cor2 = corners[0][0][0][1]
          cor1 = corners[0][0][0][0]
          id1a = corners[0][0][0][1]-corners[0][0][1][1]
          id1b = corners[0][0][0][0]-corners[0][0][1][0] 
          
          
          if cor2>= 310 and var4:
            print("Move Back")
            move (-0.05,0,0,0,0,0)
          elif cor2<= 190 and var4:
            print("Move Forward")
            move (.05,0,0,0,0,0)       
          elif cor1>= 530 and var4:
            print("Move right")
            move(0,-.05,0,0,0,0) 
          elif cor1<= 400 and var4:
            print("Move left")
            move (0,.05,0,0,0,0)
          
          elif 15>=id1a and id1b<=-2 and var4:
            print("Rotate clockwise")
            move(0,0,0,0,0,-.1)
          elif 15>=id1a and id1b>=10 and var4:
            print("Rotate clockwise")
            move(0,0,0,0,0,-.1)
          elif 25<=id1a and id1b<=-2 and var4:
            print("Rotate anticlockwise")
            move(0,0,0,0,0,.1)
          elif 25<=id1a and id1b>=10 and var4:
            print("Rotate clockwise")
            move(0,0,0,0,0,.1)
          elif 15<=id1a and id1a<=25 and id1b >=-2 and id1b <= 10 and cor1>= 400 and cor1 <= 530 and cor2>=190 and cor2<=310 and var4:
            var4=0
            print("Entered if loop",ids,cnt)
          else:
            move(0,0,0,0,0,-0.1)
            print("Auto")
          cv2.putText(cv_image, "POINT 4", tuple(corners[0][0][0]), font, 1, (0,255,0),2,cv2.LINE_AA)
            
            
        elif var4==0 and exit4==1:
          
              cnt=cnt+1
              gray = aruco.drawDetectedMarkers(cv_image, corners,ids,[0,255,255])
              move(.15,-0.09,0,0,0,0)
              print("Searching for Point 1 - Move forward",ids,cnt)
              if np.all(ids==1):
                gray = aruco.drawDetectedMarkers(cv_image, corners,ids,[0,255,255])
                print("Land")
                move(0,0,-1,0,0,0) 
                time.sleep(1)
                move(0,0,-1,0,0,0)
                land_drone()               
                exit4=0

  else:
    print("No id found move forward")
    move(0.05,0.0,0,0,0,0)
                
         
             
  # Display the resulting frame
  
  cv2.imshow('Aruco',cv_image)
  cv2.waitKey(3)


def main(args):
  rospy.init_node('image_converter', anonymous=True)
  
  try:
    n =1
    
    while(n<50000):
      cam_move(-90,0)
      
      n+=1
    takeoff_drone()
    
    
    # land_drone()
    #print("In main loop-----------------------------------------------------------------------------------------")
    
    image_sub = rospy.Subscriber("/bebop/image_raw",Image, image_callback)
    #drone_odom = rospy.Subscriber("/bebop/odom",Odometry,position)
    rospy.spin()
  except KeyboardInterrupt:

    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
  try:
    
    main(sys.argv)


  except rospy.ROSInterruptException:
    rospy.loginfo("Node Terminated")
  
