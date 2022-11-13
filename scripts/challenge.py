#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import rospy
import cv_bridge
import cv2
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from projet2022.msg import color_bound
#initialise a new ROS node called 'follow'
rospy.init_node('follow', anonymous = True)
rate = rospy.Rate(250)

global obstacle # the variable to know if we are close to the obstacle
global linear_x # linear speed
global angular_z # angular speed
global retangle
global challenge
global lower_yellow 
lower_yellow= np.array([0,0,0])
global upper_yellow 
upper_yellow=  np.array([100,100,100])
global lower_white
lower_white =  np.array([0,0,0])
global upper_white 
upper_white=  np.array([100,100,100])
global threshold # Distance minimale aux obstacles
global direction # direction = False: The  obstacle est right, il faut tourne a left
                 # direction = True: diresction de obstacle est left, il faut tourne a right

#linear_x = 0.22
linear_x = 0.1
angular_z = 0.35
#threshold = rospy.get_param("threshold")
threshold = 0.3
obstacle = False
retangle= False
# direction = False: The  obstacle est right, il faut tourne a left
# direction = True: diresction de obstacle est left, il faut tourne a right
direction = False  
challenge =1

#set the color range
lower_red = np.array([160,20,70])
upper_red = np.array([190,255,255])

#----lower_yellow = np.array([15,0,0])
#----upper_yellow = np.array([36,255,255])

#----lower_white = np.array([0,0,240])
#----upper_white = np.array([255,10,255])


# la fonction enlever les infinits et calculer le moyenne
def enleverInf(tab):
    res = [i for i in tab if i <0.35]  
    return np.mean(res) if res else 10

#La fonction détect l'obstacle et déterminer l'orientation obstacle
def callback_obsta(msg):
    global obstacle 
    global retangle
    global direction
    global threshold
    global linear_x
    global challenge
    
    # Sélectionner trois intervalle comme 'avant', 'gauche', 'droit'
    tab_front = msg.ranges[0:20] + msg.ranges[340:360]
    #tab_leftFront = msg.ranges[0:40]
    #tab_rightFront = msg.ranges[320:360]
    tab_leftFront = msg.ranges[20:50]  #############################################################################
    tab_rightFront = msg.ranges[310:340] ##########################################################################
    

    # Calculer les moyennes des trois tableaus précédent en utilisant la fonciton 'enleverInf(tab)'
    mean_front = enleverInf(tab_front)
    mean_leftFront = enleverInf(tab_leftFront)
    mean_rightFront = enleverInf(tab_rightFront)
    #print(mean_front , mean_leftFront, mean_rightFront)
    
    #Si le distance entre robot et l'obstace est inférieur que threshold, 'obstacle' = True, else 'obstacle = False'
    if mean_leftFront <threshold or mean_rightFront <threshold :#mean_front<(threshold*1.5) or
        obstacle = True
    else:
        obstacle = False
    
    
    if mean_leftFront <threshold and mean_rightFront <threshold and mean_front>threshold:
    	challenge ==2
    	  
    #calculer le minimal valeur devant de robot
    min_front = min(min(tab_front), 2)

    if challenge ==2:
        #if min_front > threshold:
        if min_front > threshold:
            pass #avancer() #pas utiliser ???????
        #if there is a obstacle in front of the robot    
        else:
        	if mean_leftFront < mean_rightFront: # if the right distance is bigger than the left, we turn to right
        		direction  = True
        	elif mean_leftFront >= mean_rightFront:  # if the right distance is smaller than the left, we turn to left
        		direction  = False
        	tourne()
       
                
    # if the mean of left is smaller than right it means that the obstacle
    # is on the left, it hace to turn on the right, 'direction' = True
    if mean_leftFront < mean_rightFront:
        direction  = True
    # if the mean of left is bigger than right it means that the obstacle
    # is on the right, it hace to turn on the left, 'direction '= False    
    elif mean_leftFront >= mean_rightFront:
        direction = False

#La fonciton qui détect le ligne
def callback_follow(msg):
    global obstacle
    global linear_x 
    global angular_z 
    global retangle
    
    #creat a bridge 
    #-----------cvBridge = cv_bridge.CvBridge ()
    #change the image to opencv and change the RGB to hsv
    #-----------cvImage = cvBridge.imgmsg_to_cv2 (msg , desired_encoding='bgr8')
    

    
    #determine the center of image
    #-----------centre_robot = width/2
    ######setColor_param(msg)
    cvBridge1 = cv_bridge.CvBridge()
    cvImage = cvBridge1.compressed_imgmsg_to_cv2 (msg)
    
    
    cv2.imshow("Image windowwwwwwwwwwwww", cvImage  )
    cv2.waitKey(3)
    
    
    
    
    hsv = cv2.cvtColor(cvImage,cv2.COLOR_BGR2HSV)
    cv2.imshow("Image windowwwwwwwcwwwwww", hsv  )
    cv2.waitKey(3)
    mask_yellow = cv2.inRange (hsv, lower_yellow, upper_yellow)
    mask_white  = cv2.inRange (hsv,lower_white,upper_white)
    # get the shape of image
    hight, width, deep = cvImage.shape
    #Cacher une partie de l'image afin que 
    #la caméra se concentre uniquement sur la piste la plus proche
    search_top = int(4*hight/6)
    #mask_yellow[0:search_top, 0:width] = 0
    #mask_white[0:search_top, 0:width] = 0
    # Afin de passer l'intersection, on cache Nous avons masqué le côté droit de l'image jaune 
    #et le côté gauche de l'image blanche, Mais les deux parties cachées ne sont pas symétriques 
    #et égales,  pour permettre au robot de tourner, on cache plus de parties sur l'image jaune 
    #que sur l'image blanche
    dis = 40
    demi_width = int(width/2)
    #mask_yellow[0:hight, (demi_width+dis):width] = 0
    #mask_white[0:hight, 0:(demi_width)] = 0
        
    #Calculer les moments
    M_yellow = cv2.moments(mask_yellow)
    M_white  = cv2.moments(mask_white )
    
    if M_yellow[ "m00" ] >0:
        cX_yellow = int (M_yellow[ "m10" ] / M_yellow[ "m00" ] )
        cY_yellow = int (M_yellow[ "m01" ] / M_yellow[ "m00" ] )
    else:
        cX_yellow =5
        cY_yellow = 180
    # draw a circle at center
    #cv2.circle(mask_yellow, (demi_width+40,10 ), 5, 50,2)
    cv2.circle(mask_yellow, (cX_yellow ,cY_yellow ), 15, 50,2)

    if M_white [ "m00" ] >0 :          
        cX_white  = int (M_white [ "m10" ] / M_white [ "m00" ] )
        cY_white  = int (M_white [ "m01" ] / M_white [ "m00" ] )
    else:
        cX_white = 315
        cY_white = 180
    # draw a circle at center
    #cv2.circle(mask_white, (demi_width,10 ), 5, 150,2)
    cv2.circle(mask_white, (cX_white ,cY_white ), 15, 150,2)
    
    #Afin d'éviter le caméra ne détectz pas le coleur ou la cas extrêmes, 
    #Nous fixons le min et le max
    #if  cX_yellow<5: ######################################################################
        #cX_yellow = 5   
    #if cX_white >315:
        #cX_white = 315
        
    centre_element = (cX_yellow + cX_white)/2
#    cv2.circle(mask_white, (int(centre_element),100), 5, 50,2)
#    cv2.circle(mask_yellow, (int(centre_element),100 ), 5, 50,2)
#    cv2.circle(mask_white, (int(centre_robot),100), 5, 150,2)
#    cv2.circle(mask_yellow, (int(centre_robot),100 ), 5, 150,2)
    
    #Calculer l'erreur entre centre robot et le centre des deux coleurs
    error = centre_element #- centre_robot

    #Afficher les images
    cv2.imshow("Image window1", mask_yellow )
    cv2.imshow("Image window2", mask_white  )
    cv2.waitKey(3)
    
    
    
    #Si il a y un obstacle en avant, on tourne
    #if obstacle:
     #   tourne()     
    #si il y a pas de l'obstacle, on suivre le ligne
    #elif not obstacle:
     #   follow(error)

def nothing(x):
    pass
    
def setColor_param(msg):
	global lower_yellow
	global upper_yellow
	global lower_white
	global upper_white
	#creat a bridge 
	cvBridge = cv_bridge.CvBridge ()
	#change the image to opencv and change the RGB to hsv
	cvImage = cvBridge.imgmsg_to_cv2 (msg , desired_encoding='bgr8')
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
	    lower_yellow = np.array([hMin_yellow, sMin_yellow, vMin_yellow])
	    upper_yellow = np.array([hMax_yellow, sMax_yellow, vMax_yellow])
	    
	    lower_white = np.array([hMin_white, sMin_white, vMin_white])
	    upper_white = np.array([hMax_white, sMax_white, vMax_white])

	    # Wait longer to prevent freeze for videos.
	    if cv2.waitKey(wait_time) & 0xFF == ord('q'):
	    	break

	cv2.destroyAllWindows() 
          
def tourne():
    global linear_x 
    global angular_z 
    data = Twist()
    #data.linear.x=linear_x *0.5
    data.linear.x=linear_x  ###################################################################################
    if direction:
        print('It turn on the right')
        #data.angular.z= - angular_z *2.8# obstacle est left,  tourne vers right 
        data.angular.z= -angular_z *1.5# obstacle est left,  tourne vers right   ###############################################
    else:
        print('It turn on the left')
        data.angular.z= angular_z*1.5 # obstacle est right,  tourne vers left  ##########################################
        #data.angular.z= angular_z*2.8 # obstacle est right,  tourne vers left
    pub.publish(data)
    rate.sleep()  
    
def follow(error):
    global linear_x 
    global angular_z 
    data = Twist() 
    #if error >30:
        #data.linear.x=0.05
        #data.angular.z=-angular_z*0.5
    #elif error<-30:
        #data.linear.x=0.05
        #data.angular.z=angular_z*0.5 
    #else:
    print('It follow the line') 
    #data.angular.z = -float(error)/150  ########################################################  
    #data.angular.z = -float(error)/100  ######################################################## 
    data.angular.z = -float(error)/60  ########################################################
    data.linear.x=linear_x ###############################################################
    #data.linear.x=linear_x *0.5  ###############################################################
    #data.linear.x=linear_x *0.3
    pub.publish(data)
    rate.sleep()    
    
def callback_color(msg):
	global lower_yellow
	global upper_yellow
	global lower_white
	global upper_white
	
	lower_yellow = np.array(msg.lower_yellow)
	upper_yellow = np.array(msg.upper_yellow)
	lower_white = np.array(msg.lower_white)
	upper_white = np.array(msg.upper_white)
           
if __name__ == '__main__':
    try:
        #create a publiser, publise a topic named 'cmd_vel',
        #type of message is Twist. The length of queue is 10
        rospy.Subscriber('/color_boundaries', color_bound, callback_color);
        
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        #define a subscriber on the /camera/image topic with a import data type before
        #together with a callback fonction
        #rospy.Subscriber("/camera/image", Image, callback_follow); #pour la simu
        rospy.Subscriber("/raspicam_node/image/compressed", CompressedImage, callback_follow); #pour la realite
        #define a subscriber on the /scan topic with a import data type before
        #together with a callback fonction
        rospy.Subscriber("/scan", LaserScan, callback_obsta);
        
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

#def avancer():
#    global linear_x 
#    global angular_z 
#    
#    data = Twist()
#    data.linear.x=0.1
#    data.angular.z = 0
#    pub.publish(data)
#    rate.sleep() 
#    
#def follow_yellow(error, centre_robot, cX_yellow, cX_white):
#    global linear_x 
#    global angular_z 
#    data = Twist()
#
##    if error >0:
##        data.linear.x=0
##        data.angular.z=-angular_z
##    elif error<-60:
##        data.linear.x=0
##        data.angular.z=angular_z    
##    else:    
#    data.angular.z = -float(error+90)/80
#    data.linear.x=linear_x 
#    pub.publish(data)
#    rate.sleep() 
#    
#def follow_white(error, centre_robot, cX_yellow, cX_white):
#    global linear_x 
#    global angular_z 
#    data = Twist()
##    if error >70:
##        data.linear.x=0
##        data.angular.z=-angular_z
##    elif error<20:
##        data.linear.x=0
##        data.angular.z=angular_z    
##    else:    
#    data.angular.z = -float(error-90)/80
#    data.linear.x=linear_x 
#    pub.publish(data)
#    rate.sleep() 
#    
#
#    
#def follow_rec(error, centre_robot, cX_yellow, cX_white):
#    global linear_x 
#    global angular_z 
#    data = Twist() 
#    if error >30:
#        data.linear.x=linear_x
#        data.angular.z=-angular_z*0.5
#    elif error<-30:
#        data.linear.x=linear_x
#        data.angular.z=angular_z*0.5    
#    else:    
#        data.angular.z = -float(error)/200
#        data.linear.x=linear_x
#    pub.publish(data)
#    rate.sleep()  
