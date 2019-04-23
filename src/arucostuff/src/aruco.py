import numpy as np
import cv2
import cv2.aruco as aruco
import matplotlib.pyplot as plt
import rospy
from geometry_msgs.msg import Twist
import math
import time
from std_srvs.srv import Empty
from cv_bridge import CvBridge, CvBridgeError
 
cap = cv2.VideoCapture(1)
bridge = CvBridge()
 
while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    #print(frame.shape) #480x640
    # Our operations on the frame come here
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters =  aruco.DetectorParameters_create()
    font = cv2.FONT_HERSHEY_SIMPLEX #font for displaying text 
    #print(parameters)
    #lists of ids and the corners beloning to each id
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    #gray = aruco.drawDetectedMarkers(frame, corners,ids,[0,255,0])

    if np.all(ids != None):
        if np.all(ids == 1):
            gray = aruco.drawDetectedMarkers(frame, corners,ids,[0,255,255])
            print(corners[0][0][0])
            cv2.putText(frame, "lANDING ZONE", tuple(corners[0][0][0]), font, 1, (0,255,0),2,cv2.LINE_AA)
 
    # Display the resulting frame
    cv2.imshow('frame',gray)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
 
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()