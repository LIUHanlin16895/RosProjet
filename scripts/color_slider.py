#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import rospy
import cv_bridge
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from projet2022.msg import color_bound
from sensor_msgs.msg import CompressedImage

#initialise a new ROS node called 'follow'
rospy.init_node('color_slider', anonymous = True)
#rate = rospy.Rate(250)

def set_color_slider(msg):
	global lower_yellow
	global upper_yellow
	global lower_white
	global upper_white
	
	color_bound1=color_bound()
	color_bound1.lower_yellow = np.array([15,0,0])
	color_bound1.upper_yellow = np.array([63,255,255])
	color_bound1.lower_white = np.array([0,0,240])
	color_bound1.upper_white = np.array([255,10,255])
	
	#creat a bridge 
	cvBridge = cv_bridge.CvBridge ()
	#change the image to opencv and change the RGB to hsv
	#cvImage = cvBridge.imgmsg_to_cv2 (msg , desired_encoding='bgr8')  #pour la simu
	cvImage = cvBridge.compressed_imgmsg_to_cv2 (msg)   #pour le reel
	image = cv2.imread('cvImage')

	# Create a window
	cv2.namedWindow('image')

	# get the shape of image
	hight, width, deep = cvImage.shape

	#determine the center of image
	centre_robot = width/2

	hsv = cv2.cvtColor(cvImage,cv2.COLOR_BGR2HSV)

	# create trackbars for color change
	cv2.createTrackbar('HMin_yellow','image',0,179,nothing) # Hue is from 0-179 for Opencv
	cv2.createTrackbar('SMin_yellow','image',0,255,nothing)
	cv2.createTrackbar('VMin_yellow','image',0,255,nothing)
	cv2.createTrackbar('HMax_yellow','image',0,179,nothing)
	cv2.createTrackbar('SMax_yellow','image',0,255,nothing)
	cv2.createTrackbar('VMax_yellow','image',0,255,nothing)

	cv2.createTrackbar('HMin_white','image',0,179,nothing) # Hue is from 0-179 for Opencv
	cv2.createTrackbar('SMin_white','image',0,255,nothing)
	cv2.createTrackbar('VMin_white','image',0,255,nothing)
	cv2.createTrackbar('HMax_white','image',0,179,nothing)
	cv2.createTrackbar('SMax_white','image',0,255,nothing)
	cv2.createTrackbar('VMax_white','image',0,255,nothing)

	# Set default value for MAX HSV trackbars.
	cv2.setTrackbarPos('HMax_yellow', 'image', 36)
	cv2.setTrackbarPos('SMax_yellow', 'image', 255)
	cv2.setTrackbarPos('VMax_yellow', 'image', 255)
	cv2.setTrackbarPos('HMin_yellow', 'image', 15)
	cv2.setTrackbarPos('SMin_yellow', 'image', 0)
	cv2.setTrackbarPos('VMin_yellow', 'image', 0)

	cv2.setTrackbarPos('HMax_white', 'image', 179)
	cv2.setTrackbarPos('SMax_white', 'image', 10)
	cv2.setTrackbarPos('VMax_white', 'image', 255)
	cv2.setTrackbarPos('HMin_white', 'image', 0)
	cv2.setTrackbarPos('SMin_white', 'image', 0)
	cv2.setTrackbarPos('VMin_white', 'image', 240)


	# Initialize to check if HSV min/max value changes
	hMin_yellow = sMin_yellow = vMin_yellow = hMax_yellow = sMax_yellow = vMax_yellow = 0
	phMin_yellow = psMin_yellow = pvMin_yellow = phMax_yellow = psMax_yellow = pvMax_yellow = 0

	hMin_white = sMin_white = vMin_white = hMax_white = sMax_white = vMax_white = 0
	phMin_white = psMin_white = pvMin_white = phMax_white = psMax_white = pvMax_white = 0

	output = image
	wait_time = 33

	while(1):

	    # get current positions of all trackbars
	    hMin_yellow = cv2.getTrackbarPos('HMin_yellow','image')
	    sMin_yellow = cv2.getTrackbarPos('SMin_yellow','image')
	    vMin_yellow = cv2.getTrackbarPos('VMin_yellow','image')

	    hMax_yellow = cv2.getTrackbarPos('HMax_yellow','image')
	    sMax_yellow = cv2.getTrackbarPos('SMax_yellow','image')
	    vMax_yellow = cv2.getTrackbarPos('VMax_yellow','image')
	    
	    
	    hMin_white = cv2.getTrackbarPos('HMin_white','image')
	    sMin_white = cv2.getTrackbarPos('SMin_white','image')
	    vMin_white = cv2.getTrackbarPos('VMin_white','image')

	    hMax_white = cv2.getTrackbarPos('HMax_white','image')
	    sMax_white = cv2.getTrackbarPos('SMax_white','image')
	    vMax_white = cv2.getTrackbarPos('VMax_white','image')

	    # Set minimum and max HSV values to display
	    lower_yellow = [hMin_yellow, sMin_yellow, vMin_yellow]
	    upper_yellow = [hMax_yellow, sMax_yellow, vMax_yellow]
	    
	    lower_white = [hMin_white, sMin_white, vMin_white]
	    upper_white = [hMax_white, sMax_white, vMax_white]
	    
	    color_bound1.lower_yellow = lower_yellow
	    color_bound1.upper_yellow = upper_yellow
	    color_bound1.lower_white = lower_white
	    color_bound1.upper_white = upper_white
	    
	    print(color_bound1)
	    pub.publish(color_bound1)

	    # Wait longer to prevent freeze for videos.
	    if cv2.waitKey(wait_time) & 0xFF == ord('q'):
	    	break
	
	cv2.destroyAllWindows() 

def nothing(x):
    pass
    
if __name__ == '__main__':
    try:
        #create a publiser, publise a topic named 'cmd_vel',
        #type of message is Twist. The length of queue is 10
        pub = rospy.Publisher('/color_boundaries', color_bound, queue_size = 1)
        #define a subscriber on the /camera/image topic with a import data type before
        #together with a callback fonction
        #rospy.Subscriber("/camera/image", Image, set_color_slider); #pour la simu
        rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, set_color_slider); #pour la realite

        rospy.spin()

    except rospy.ROSInterruptException:
        pass


