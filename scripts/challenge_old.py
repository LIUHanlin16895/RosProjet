#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function
import numpy as np
from statistics import mean
import rospy
import cv_bridge
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
#initialise a new ROS node called 'follow'
rospy.init_node('follow', anonymous = True);
rate = rospy.Rate(100)


global obstacle # the variable to know if we are close to the obstacle
global intersection
global full_mask #the variable which decide if we need full mask in image
global linear_x 
global angular_z # linear speed and angular speed
global retangle
global yellow_left
global white_right
global threshold
global flag_front 
global flag_leftFront
global flag_rightFront
global direction

#linear_x = 0.22
linear_x = 0.1
angular_z = 0.6
full_mask = False
intersection = False
obstacle = False
retangle= False
# the min distance betwen the robot and the obstacle
threshold = 0.3 #rospy.get_param("threshold")
flag_front = False
flag_leftFront= False
flag_rightFront= False
# False: diresction de obstacle est right, il faut tourne a left
# True: diresction de obstacle est left, il faut tourne a right
direction = False  

#set the color range
lower_red = np.array([160,20,70])
upper_red = np.array([190,255,255])

lower_yellow = np.array([15,0,0])
upper_yellow = np.array([36,255,255])

lower_white = np.array([0,0,240])
upper_white = np.array([255,10,255])


# la fonction enlever les infinit
def enleverInf(tab):
    res = []
    for i in tab:
        if i<100:
            res.append(i)
    return mean(res) if res else 100

#def checkSonarStatu(min_front, min_leftFront, min_rightFront, mean_leftFront, mean_rightFront):
#    global flag_front 
#    global flag_leftFront
#    global flag_rightFront
#    
#    if min_front<threshold :
#        flag_front = True
#    else:
#        flag_front =False
#        
#    if min_leftFront<threshold :
#        flag_leftFront = True
#    else:
#        flag_leftFront =False
#        
#    if min_rightFront <threshold :
#        flag_rightFront= True
#    else:
#        flag_rightFront =False
#  
#    if flag_leftFront and not flag_front and not flag_rightFront:
#        print("left warn,turn right")
#        publisher_obsta(0, -angular_z)
#    elif flag_front and not flag_rightFront and not flag_leftFront:
#        print("front warn, left and right ok, compare left and right value to turn")
#        if mean_leftFront>=mean_rightFront:
#            print("turn left")
#            publisher_obsta(0, angular_z)
#        else:
#            print("turn right")
#            publisher_obsta(0, -angular_z)
#    elif flag_rightFront and not flag_front and not flag_leftFront:
#        print("right warn,turn left ")
#        publisher_obsta(0, angular_z)
##    elif flag_rightFront and not flag_front and not flag_leftFront:
##        print("left,front,right all warn, turn back")
##        publisher_obsta(0, 10*angular_z)
#    elif flag_leftFront and flag_front and not flag_rightFront:
#        print("left warn, front warn, right ok, turn right")
#        publisher_obsta(0, (-angular_z*2))
#    elif flag_rightFront and flag_front and not flag_leftFront:
#        print("left ok, front warn, right warn, turn left")
#        publisher_obsta(0, (angular_z*2))
#    elif flag_rightFront and flag_leftFront and not flag_front:
#        print("left and right warn, front ok, speed up")
#        publisher_obsta(linear_x*2, 0)


def callback_obsta(msg):
    global obstacle # the variable to know if we are close to the obstacle
    global intersection
    global retangle
    global direction
    global threshold
    
#    tab_front = msg.ranges[0:30] + msg.ranges[330:360]
    tab_leftFront = msg.ranges[0:35]
    tab_rightFront = msg.ranges[325:360]
    tab_front = tab_leftFront + tab_rightFront 
    
    min_front = min(tab_front)
#    min_leftFront = min(tab_leftFront )
#    min_rightFront= min(tab_rightFront)
    
    mean_leftFront = enleverInf(tab_leftFront)
    mean_rightFront = enleverInf(tab_rightFront)
    # si la valeur minimal est inférieur que thresold; il y a obstacle
    if min_front <threshold:
        obstacle = True
        print('obstacle est True')
    else:
        # si non , il y a pas de obstacle
        obstacle = False
        print('obstacle est False')
    # if the mean of left is smaller than right it means that the obstacle
    # is on the left, it hace to turn on the right, direction = True
    if mean_leftFront < mean_rightFront:
        direction  = True
    # if the mean of left is bigger than right it means that the obstacle
    # is on the right, it hace to turn on the left, direction = False    
    elif mean_leftFront >= mean_rightFront:
        direction = False
    # rectangle vérifier si on pass l'obsrtacle rectangle, si on pass rectangle = false
    if not obstacle: # il n y a pas de obstacle devant nous 
        if min(msg.ranges[280:360]) >0.4 or min(msg.ranges[0:80]) >0.4:
            retangle = False # pass le rectangle, on peut reutilise l'algotrhiteme follow
        
#    checkSonarStatu(min_front, min_leftFront, min_rightFront, \
#                    mean_leftFront, mean_rightFront)
        
def callback_follow(msg):
    global obstacle # the variable to know if we are close to the obstacle
    global intersection
    global full_mask #the variable which decide if we need full mask in image
    global linear_x 
    global angular_z # linear speed and angular speed
    global retangle
    global yellow_left
    global white_right
    
    #creat a bridge 
    cvBridge = cv_bridge.CvBridge ()
    #change the image to opencv and change the RGB to hsv
    cvImage = cvBridge.imgmsg_to_cv2 (msg , desired_encoding='bgr8')
    
    # get the shape of image
    hight, width, deep = cvImage.shape
    search_top = int(4*hight/6)
    demi_width = int(width/2)
    #determine the center of image
    centre_robot = width/2
    
    hsv = cv2.cvtColor(cvImage,cv2.COLOR_BGR2HSV)
    mask_yellow = cv2.inRange ( hsv, lower_yellow, upper_yellow)
    mask_white  = cv2.inRange (hsv,lower_white,upper_white)
    mask_yellow2 = cv2.inRange ( hsv, lower_yellow, upper_yellow)
    mask_white2  = cv2.inRange (hsv,lower_white,upper_white)
    
#    if retangle:full_mask =True
    if not full_mask:
        dis = 40
        mask_yellow[0:search_top, 0:width] = 0
        mask_yellow[0:hight, (demi_width+dis):width] = 0
        mask_white[0:search_top, 0:width] = 0
        mask_white[0:hight, 0:(demi_width)] = 0
        
    M_yellow = cv2.moments(mask_yellow)
    M_white  = cv2.moments(mask_white )

    if M_yellow[ "m00" ] >0  and M_white [ "m00" ] >0 :

        cX_yellow = int (M_yellow[ "m10" ] / M_yellow[ "m00" ] )
        cY_yellow = int (M_yellow[ "m01" ] / M_yellow[ "m00" ] )
        # draw a circle at center
        cv2.circle(mask_yellow, (demi_width+40,10 ), 5, 50,2)
        cv2.circle(mask_yellow, (cX_yellow ,cY_yellow ), 15, 50,2)
        cv2.circle(mask_yellow2, (cX_yellow ,cY_yellow ), 15, 50,2)

          
        cX_white  = int (M_white [ "m10" ] / M_white [ "m00" ] )
        cY_white  = int (M_white [ "m01" ] / M_white [ "m00" ] )
        # draw a circle at center
        cv2.circle(mask_white, (demi_width,10 ), 5, 150,2)
        cv2.circle(mask_white, (cX_white ,cY_white ), 15, 150,2)
        cv2.circle(mask_white2, (cX_white ,cY_white ), 15, 150,2)

        centre_element = (cX_yellow + cX_white)/2
    #    print("centre des deux centres d'element est", centre_element)
        error = centre_element - centre_robot
        
        cv2.imshow("Image window1", mask_yellow )
        cv2.imshow("Image window2", mask_white  )
#        cv2.imshow("Image window3", mask_yellow + mask_white)
#        cv2.imshow("Image window4", mask_yellow2 + mask_white2)
        cv2.waitKey(3)
    
        if obstacle:
            tourne()
            retangle = True
        elif retangle and not obstacle:
            avancer()
#        elif intersection and not obstacle:
#            pass
#        elif not intersection and not retangle:
        elif not intersection and not retangle and not obstacle:
            
            follow(error, centre_robot,cX_yellow , cX_white)
    

def tourne():
    global linear_x 
    global angular_z 
    data = Twist()
    data.linear.x=0
    if direction:
        print('il tourne')
        data.angular.z= - angular_z   # obstacle est left,  tourne vers right 
    elif not direction:
        data.angular.z= angular_z  # obstacle est right,  tourne vers left
    pub.publish(data)
    rate.sleep()  
#    data.angular.z= 0
#    pub.publish(data)
#    rate.sleep()  
        
def avancer():
    global linear_x 
    global angular_z 
    print('il avance')
    data = Twist()
    data.linear.x=linear_x 
    data.angular.z = 0
    pub.publish(data)
    rate.sleep() 

def follow(error, centre_robot, cX_yellow, cX_white):
    global linear_x 
    global angular_z 
    print('il follow')
    data = Twist()
#    if cX_yellow>centre_robot or cX_white<centre_robot:
#        for i in range(5):
#            data.angular.z=angular_z  
#            pub.publish(data)
#            rate.sleep()  
    if error >30:
        data.linear.x=0
        data.angular.z=-angular_z
    elif error<-30:
        data.linear.x=0
        data.angular.z=angular_z    
    else:    
        data.angular.z = -float(error)/80
        data.linear.x=0.1
    pub.publish(data)
    rate.sleep()    

#def publisher_obsta(linear_x, angular_z):
#    #create a publiser, publise a topic named 'cmd_vel',
#    #type of message is Twist. The length of queue is 10
#    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
#    rate =rospy.Rate(10)
#    # robot go follow the line
#    data = Twist()
#    data.linear.x=linear_x
#    data.linear.y=0
#    data.linear.z=0
#    data.angular.x=0 
#    data.angular.y=0 
#    data.angular.z =angular_z
#    pub.publish(data)
#    rate.sleep()  
       
        
if __name__ == '__main__':
    try:
        #create a publiser, publise a topic named 'cmd_vel',
        #type of message is Twist. The length of queue is 10
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        #define a subscriber on the /camera/image topic with a import data type before
        #together with a callback fonction
        rospy.Subscriber("/camera/image", Image, callback_follow);
        #define a subscriber on the /scan topic with a import data type before
        #together with a callback fonction
        rospy.Subscriber("/scan", LaserScan, callback_obsta);
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
#    if M_yellow[ "m00" ] >0 :
#
#        cX_yellow = int (M_yellow[ "m10" ] / M_yellow[ "m00" ] )
#        cY_yellow = int (M_yellow[ "m01" ] / M_yellow[ "m00" ] )
#        # draw a circle at center
#        cv2.circle(mask_yellow , (cX_yellow ,cY_yellow ), 15, 150,2)
#
#          
#    if M_white [ "m00" ] >0 :
#        cX_white  = int (M_white [ "m10" ] / M_white [ "m00" ] )
#        cY_white  = int (M_white [ "m01" ] / M_white [ "m00" ] )
#        # draw a circle at center
#        cv2.circle(mask_white , (cX_white ,cY_white ), 15, 150,2)
#
#    centre_element = (cX_yellow + cX_white)/2
##    print("centre des deux centres d'element est", centre_element)
#    error = centre_element - centre_robot
#    
#    cv2.imshow("Image window1", mask_yellow )
#    cv2.imshow("Image window2", mask_white  )
#    cv2.imshow("Image window3", mask_yellow + mask_white)
#    cv2.imshow("Image window4", mask_yellow2 + mask_white2)
#    cv2.waitKey(3)
#
#    if obstacle:
#        tourne()
#        retangle = True
#    elif retangle and not obstacle:
#        avancer()
#    elif intersection and not obstacle:
#        pass
#    elif not intersection and not retangle:
#        follow(error)
