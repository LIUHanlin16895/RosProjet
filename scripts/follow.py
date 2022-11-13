#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function
import rospy
import numpy as np
from statistics import mean
from sensor_msgs.msg import Image
import cv_bridge
import cv2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

global lost_yellow_count
global lost_white_count
lost_yellow_count = 0
lost_white_count = 0
global threshold
global rectangle
global direction
global obstacle # the variable to know if we are close to the obstacle
global intersection
global full_mask #the variable which decide if we need full mask in image
global linear_x 
global angular_z # linear speed and angular speed
global yellow_left
global white_right
global flag_front 
global flag_leftFront
global flag_rightFront
global skip_cam

#linear_x = 0.22
linear_x = 0.1
angular_z = 0.35
full_mask = False
intersection = False
obstacle = False
rectangle= False
# the min distance betwen the robot and the obstacle
threshold = 0.3 #rospy.get_param("threshold")
flag_front = False
flag_leftFront= False
flag_rightFront= False
# False: diresction de obstacle est right, il faut tourne a left
# True: diresction de obstacle est left, il faut tourne a right
direction = False
skip_cam = 0


# Callback function for reading turtlesim node output
def read_pose_callback(msg):
    global lost_yellow_count
    global lost_white_count
    global skip_cam

    visual_treshold=1
    m=0
    
    lower_red=np.array([160,20,70])
    upper_red=np.array([190,255,255])

    lower_yellow=np.array([15,0,0])
    upper_yellow=np.array([63,255,255])

    lower_white=np.array([0,0,240])
    upper_white=np.array([255,10,255])
    

    cvBridge = cv_bridge.CvBridge()
    # Transform the image to openCV format, msg is the original image from ROS
    cvImage = cvBridge.imgmsg_to_cv2(msg , desired_encoding='bgr8')
    # Change color representation from BGR to HSV
    hsv = cv2.cvtColor(cvImage , cv2.COLOR_BGR2HSV)
    
    # get the shape of image
    hight, width, deep = cvImage.shape
    # cvImage.shape[0] =hight/2
    # cvImage.shape[1] =width/2
    
    bottom_area=round(hight/3)
    hight=round(hight)
    width=round(width)
    deep=round(deep)
    
    # Image binarisation
    mask_red = cv2.inRange( hsv ,lower_red, upper_red)
    mask_yellow = cv2.inRange( hsv , lower_yellow, upper_yellow)
    mask_white = cv2.inRange( hsv, lower_white, upper_white)

    #mask1 = cv2.bitwise_or(mask_red,mask_yellow)
    #mask2 = cv2.bitwise_or(mask_red,mask_white)
    mask_ry=mask_red+mask_yellow
    mask=cv2.bitwise_or(mask_ry,mask_white)
    # Compute the mask moments
    
    #mask_yellow[bottom_area:hight-1,0:width-1] = 0
    #mask_white[bottom_area:hight-1,0:width-1] = 0
    
    #mask_yellow[0:bottom_area,0:width-1] = 0
    #mask_white[0:bottom_area,0:width-1] = 0
    
    mask_yellow[0:round(3*hight/4),0:width-1] = 0
    #mask_yellow[round((3*hight/4)+20):hight,0:width-1] = 0
    mask_white[0:round(3*hight/4),0:width-1] = 0
    #mask_white[round((3*hight/4)+20):hight,0:width-1] = 0
    
    #mask[bottom_area:hight,0:width] = 0
    #mask_yellow[0,0]=0
    #mask_yellow[hight-1,width-1]=0
    #print(mask_yellow)
    
    
    M_red = cv2.moments(mask_red)
    if M_red["m00"]>0 :
        # Calculate x , y coordinate of center
        cX_red = int(M_red["m10"]/M_red["m00"])
        cY_red = int(M_red["m01"]/ M_red["m00"] )
        #Display the image with cv2.imshow, or draw a form with cv2.rectangle or cv2.circle
        #print(cX, cY)

    M_yellow = cv2.moments(mask_yellow)
    if M_yellow["m00"]>0 :
        # Calculate x , y coordinate of center
        cX_yellow = int(M_yellow["m10"]/M_yellow["m00"] )
        cY_yellow = int(M_yellow["m01"]/ M_yellow["m00"] )
	
	# draw a circle at center
        cv2.circle(mask_yellow , (cX_yellow ,cY_yellow ), 15, 150,2)
        #print('centre du contour jaune est ', cX_yellow , cY_yellow )
        lost_yellow_count =0
        
    else:  #si aucun trait jaune n'est detecté
    	cX_yellow = round(width/2)
    	cY_yellow = round(hight/2)
    	
    	lost_yellow_count += 1


    M_white = cv2.moments(mask_white)
    if M_white["m00"]>0 :
        # Calculate x , y coordinate of center
        cX_white = int(M_white["m10"]/M_white["m00"] )
        cY_white = int(M_white["m01"]/ M_white["m00"] )
        
        # draw a circle at center
        cv2.circle(mask_white , (cX_white ,cY_white ), 15, 150,2)
        #print('centre du contour blanc est ', cX_white , cY_white )
    else:
    	cX_white = round(width/2)
    	cY_white = round(hight/2)
    	
    	lost_white_count += 1
    		

    M = cv2.moments(mask)
    if M["m00"]>0 :
    	# Calculate x , y coordinate of center
    	cX = int(M["m10"]/M["m00"])
    	cY = int(M["m01"]/ M["m00"] )
    	cv2.circle(mask , (cX_yellow ,cY_yellow ), 15, 150, 2)
    	cv2.circle(mask , (cX_white ,cY_white ), 15, 150, 2)
    
    #determine the center of image
    centre_robot = width/2
    
    centre_element = (cX_yellow + cX_white)/2
    #print("centre des deux centres d'element est", centre_element)
    
    cv2.circle(mask , (round(centre_element) ,round(width/2) ), 7, 150,2)
    
    
    if lost_yellow_count < visual_treshold and lost_white_count < visual_treshold:
    	
    	if cX_white < cX_yellow:
    		mask_yellow[0:round(hight)-1,round(width/3):width] = 0
    		M_yellow = cv2.moments(mask_yellow)
    		if M_yellow["m00"]>0 :
    			# Calculate x , y coordinate of center
    			cX_yellow = int(M_yellow["m10"]/M_yellow["m00"] )
    			cY_yellow = int(M_yellow["m01"]/ M_yellow["m00"] )
    			
    			# draw a circle at center
    			cv2.circle(mask_yellow , (cX_yellow ,cY_yellow ), 15, 150,2)
    			#print('centre du contour jaune est ', cX_yellow , cY_yellow )
    			lost_yellow_count =0
    		
    		else:  #si aucun trait jaune n'est detecté
    			cX_yellow = round(width/2)
    			cY_yellow = round(hight/2)
    			
    			lost_yellow_count += 1
    			m=1 #just an indicator to see if the else has been run in order to know if we execute the next lines
    	
    	if m==0:
    		#peut etre mettre la suite dans le 3eme if imbriqué (cf juste au dessus)
    		delta_p = abs(cX_white - cX_yellow)
    		if delta_p > width/2:
    			error = (cX_yellow + (width/8)) - centre_robot #pour être un peu a droite de la ligne jaune
    		else:
    			error = centre_element - centre_robot
    		if skip_cam==0:	
    			publisher_follow(error)
    	
    #cv2.imshow("Image window red", mask_red)
    cv2.imshow("Image window yellow", mask_yellow)
    cv2.imshow("Image window white", mask_white)
    cv2.imshow("Image window", mask)
    
    if lost_yellow_count > visual_treshold and lost_white_count <= visual_treshold:
    	#pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
    	#rate =rospy.Rate(100)
    	
    	#data = Twist()
    	#data.linear.x=0
    	#data.angular.z = -np.pi/6
    	#pub.publish(data)
    	#rate.sleep()
    	
    	error = (cX_white - (width/6)) - centre_robot
    	
    	if skip_cam==0:
    		publisher_follow(error)
    
    if lost_yellow_count <= visual_treshold and lost_white_count > visual_treshold:
    	#pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
    	#rate =rospy.Rate(100)
    	
    	#data = Twist()
    	#data.linear.x=0
    	#data.angular.z = np.pi/6
    	#pub.publish(data)
    	#rate.sleep()
    	error = (cX_yellow + (width/6)) - centre_robot
    	
    	if skip_cam==0:
    		publisher_follow(error)
    
    if lost_yellow_count > visual_treshold and lost_white_count > visual_treshold:
    	pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
    	rate =rospy.Rate(100)
    	
    	data = Twist()
    	data.linear.x=0
    	if lost_yellow_count < lost_white_count:
    		data.angular.z = np.pi/6
    	else:
    		data.angular.z = -np.pi/6
    	if skip_cam==0:
    		pub.publish(data)
    		rate.sleep()

    	
    cv2.waitKey(3)

def publisher_follow(error):
    #create a publiser, publise a topic named 'cmd_vel',
    #type of message is Twist. The length of queue is 10
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
    rate =rospy.Rate(100)
    # robot go follow the line
    data = Twist()
    data.angular.z = -float(error)/60
    data.linear.x=0.1
    
    pub.publish(data)
    rate.sleep() 
    
def publisher_obsta():
    #create a publiser, publise a topic named 'cmd_vel',
    #type of message is Twist. The length of queue is 10
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
    rate =rospy.Rate(100)
    # robot go follow the line
    data = Twist()
    data.linear.x=0
    data.angular.z =0
    pub.publish(data)
    rate.sleep() 

def callback_obsta(msg):
    global obstacle # the variable to know if we are close to the obstacle
    global intersection
    global rectangle
    global direction
    global threshold
    global skip_cam
    
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
    # if the mean of left is smaller than right it means that the obstacle
    # is on the left, it hace to turn on the right, 'direction' = True
    if mean_leftFront < mean_rightFront:
        direction  = True
    # if the mean of left is bigger than right it means that the obstacle
    # is on the right, it hace to turn on the left, 'direction '= False    
    elif mean_leftFront >= mean_rightFront:
        direction = False
        
    if not obstacle: # il n y a pas de obstacle devant nous 
    	skip_cam =0
    	#if min(msg.ranges[280:360]) >0.4 or min(msg.ranges[0:80]) >0.4:
        	#rectangle = False # pass le rectangle, on peut reutilise l'algotrhiteme follow
    if obstacle:
    	tourne()
    	skip_cam =1
    	print("instruction")
#    checkSonarStatu(min_front, min_leftFront, min_rightFront, \
#                    mean_leftFront, mean_rightFront)
    
		
def tourne():
    global linear_x 
    global angular_z 
    global direction
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
    rate =rospy.Rate(100)
    data = Twist()
    data.linear.x=0
    if direction:
        print('il tourne a droite')
        data.angular.z= - angular_z*1.5   # obstacle est left,  tourne vers right 
    elif not direction:
    	print('il tourne a droite')
    	data.angular.z= angular_z*1.5  # obstacle est right,  tourne vers left
    pub.publish(data)
    rate.sleep()  
        	

#la fonction enlever les infinit
def enleverInf(tab):
    res = [i for i in tab if i <0.35]  
    return np.mean(res) if res else 10

if __name__ == '__main__':
    try:
        
        # CODE TO BE COMPLETED
        rospy.init_node('follow', anonymous=True)
        
        #mysub=rospy.Subscriber ("/camera/image",Image,read_pose_callback)
        #mypub=rospy.Subscriber("/scan", LaserScan, callback_obsta);
        
        rospy.Subscriber("/scan", LaserScan, callback_obsta);
        
        if skip_cam ==0:
        	rospy.Subscriber ("/camera/image",Image,read_pose_callback)
			
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
