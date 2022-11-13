#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import rospy
import cv_bridge
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
#initialise a new ROS node called 'corridor'
rospy.init_node('corridor', anonymous = True)
rate = rospy.Rate(10)

global obstacle # the variable to know if we are close to the obstacle
global linear_x # linear speed
global angular_z # angular speed
global threshold # Distance minimale aux obstacles
global direction # direction = False: The  obstacle est right, il faut tourne a left
                 # direction = True: diresction de obstacle est left, il faut tourne a right
global yellow  # 'Yellow = True': it means that if the camera can see the yellow line
global white   # 'white = True': it means that if the camera can see the white line
global red     # 'res = True': it means that if the camera can see the red things
global challenge # determine on est dans quelle challenge
global sorti

#linear_x = 0.22
linear_x = 0.1
angular_z = 0.35
#threshold = rospy.get_param("threshold")
threshold = 0.5
challenge = 1
obstacle = False
# direction = False: The  obstacle est right, il faut tourne a left
# direction = True: diresction de obstacle est left, il faut tourne a right
direction = False  
yellow = False
white = False
red = False
sorti = False

#set the color range
lower_red = np.array([160,20,70])
upper_red = np.array([190,255,255])

lower_yellow = np.array([15,0,0])
upper_yellow = np.array([36,255,255])

lower_white = np.array([0,0,240])
upper_white = np.array([255,10,255])

data = Twist() 

# la fonction enlever les infinits et calculer le moyenne
#def enleverInf(tab):
#    res = [i for i in tab if i <0.35]  
#    return np.mean(res) if res else 10
def enleverZer(tab):
    res = [i for i in tab if i != 0]  
    return np.min(res) if res else 10

def callback_obsta(msg):
    global threshold
    global challenge 
    global direction
    global obstacle
    global sorti
    
    data = Twist() 
    
    linear_x = 0
    angular_z = 0
    
    # Sélectionner trois intervalle comme 'avant', 'gauche', 'droit'
    tab_front = msg.ranges[0:20] + msg.ranges[340:360]
#    tab_front = msg.ranges[0:30] + msg.ranges[330:360]
    tab_leftFront = msg.ranges[10:40]  #############################################################################
    tab_rightFront = msg.ranges[310:350] ##########################################################################
#    tab_leftFront = msg.ranges[10:120]  #############################################################################
#    tab_rightFront = msg.ranges[240:350] ##########################################################################

    # Calculer les moyennes des trois tableaus précédent en utilisant la fonciton 'enleverInf(tab)'
    #mean_front = enleverInf(tab_front)
    #mean_leftFront = enleverInf(tab_leftFront)
    #mean_rightFront = enleverInf(tab_rightFront)
    
    mean_front = enleverZer(tab_front)
    mean_leftFront = enleverZer(tab_leftFront)
    mean_rightFront = enleverZer(tab_rightFront)
    
#    mean_leftFront = np.mean(tab_leftFront)
#    mean_rightFront = np.mean(tab_rightFront)
#    if mean_leftFront < 0.3 and mean_rightFront <0.3 and not yellow and not white:
#        challenge = 2
#        print('Entrer challenge 2 !!!!!')
#    else:
#        challenge = 1
#        print('Challenge 1')
    #calculer le minimal valeur devant de robot
    min_front = min(min(tab_front), 1)
     
    if mean_front > 0.3:
        linear_x = 0.18
        angular_z = 0
        if min(msg.ranges[0:60] + msg.ranges[300:360])>1:
            linear_x = 0
            angular_z = 0
    else:
        if mean_leftFront > mean_rightFront:
            linear_x = 0.02
            #angular_z = 0.8
            angular_z = 0.6
        elif mean_leftFront <= mean_rightFront:
            linear_x = 0.02
            #angular_z = -0.8
            angular_z = -0.6
        
    data.linear.x = linear_x
    data.angular.z = angular_z 
    pub.publish(data)
    rate.sleep()    
            
        
    
    
#    # if the mean of left is smaller than right it means that the obstacle
#    # is on the left, it hace to turn on the right, 'direction' = True
#    if mean_leftFront < mean_rightFront:
#        direction  = True
#    # if the mean of left is bigger than right it means that the obstacle
#    # is on the right, it hace to turn on the left, 'direction '= False    
#    elif mean_leftFront >= mean_rightFront:
#        direction = False
        
        
#    if obstacle:
#        tourne()     
#    #si il y a pas de l'obstacle, on suivre le ligne
#    elif not obstacle:
#        avancer()  
                
        

def avancer():
    global linear_x 
    global angular_z 
    data = Twist() 
    data.linear.x = linear_x
    data.angular.z = 0
    pub.publish(data)
    rate.sleep()

def tourne():
    global linear_x 
    global angular_z 
    data = Twist()
    #data.linear.x=linear_x *0.5
    data.linear.x=linear_x  ###################################################################################
    if direction:
#        print('It turn on the right')#####################
        #data.angular.z= - angular_z *2.8# obstacle est left,  tourne vers right 
        data.angular.z= -angular_z*2.2# obstacle est left,  tourne vers right   ###############################################
    else:
#        print('It turn on the left') #####################
        data.angular.z= angular_z*2.2 # obstacle est right,  tourne vers left  ##########################################
        #data.angular.z= angular_z*2.8 # obstacle est right,  tourne vers left
    pub.publish(data)
    rate.sleep()   



def callback_follow(msg):
    global obstacle
    global linear_x 
    global angular_z 
    global yellow  # 'Yellow = True': it means that if the camera can see the yellow line
    global white   # 'white = True': it means that if the camera can see the white line
    global red     # 'res = True': it means that if the camera can see the red things
    global challenge
    
    yellow = False
    white = False
    #creat a bridge 
    cvBridge = cv_bridge.CvBridge ()
    #change the image to opencv and change the RGB to hsv
    cvImage = cvBridge.imgmsg_to_cv2 (msg , desired_encoding='bgr8')
    
    # get the shape of image
    hight, width, deep = cvImage.shape
    
    hsv = cv2.cvtColor(cvImage,cv2.COLOR_BGR2HSV)
    mask_yellow = cv2.inRange (hsv, lower_yellow, upper_yellow)
    mask_white  = cv2.inRange (hsv,lower_white,upper_white)
    mask_red = cv2.inRange (hsv,lower_red,upper_red)
    
    #Cacher une partie de l'image afin que 
    #la caméra se concentre uniquement sur la piste la plus proche
    search_top = int(4*hight/6)
    #mask_red[0:search_top, 0:width] = 0
        
    #Calculer les moments
    M_yellow = cv2.moments(mask_yellow)
    
    M_white  = cv2.moments(mask_white )
    M_red = cv2.moments(mask_red)
    
    if M_yellow["m00"]>0:
#        print('yellow = true')
        yellow = True    # Si le camera peut detecter le jaune
        cX_yellow = int(M_yellow["m10"] / M_yellow["m00"])
        cY_yellow = int(M_yellow["m01"] / M_yellow["m00"])
    else:
#        print('yellow = False')
        yellow = False
        cX_yellow =5
        cY_yellow = 180

    if M_white["m00"]>0 : 
#        print('white = true')
        white = True     # Si le camera peut detecter le blanche 
        cX_white  = int(M_white["m10"] / M_white ["m00"])
        cY_white  = int(M_white["m01"] / M_white ["m00"])     
    else:
#        print('white = False')
        white = False
        cX_white = 315
        cY_white = 180
    
#    if  not white and not yellow and obstacle:
#        challenge = 2
#        print('Entrer challenge 2 !!!!!')
#    #Si il a y un obstacle en avant, on tourne
#    else:
#        challenge = 1  
#        print('detecter les coleurs')
        
        #print('entre challenge 2 ou 3 !!!!!')
    # Si le camera peut detecter ligne joune ou ligne blanche, on suivre le ligne
    # Si non, on est dans le challenge 2 ou challenge 3 
#    if not white and not yellow and obstacle:
#        challenge = 2
#        print('Entrer challenge 2 !!!!!')
#        if obstacle:
#            tourne()     
##    #si il y a pas de l'obstacle, on suivre le ligne
#        elif not obstacle:
#            avancer() 
#    else:
#        challenge = 1
#        print('detecter les coleurs')
        
        

if __name__ == '__main__':
    try:
        #create a publiser, publise a topic named 'cmd_vel',
        #type of message is Twist. The length of queue is 10
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        #define a subscriber on the /camera/image topic with a import data type before
        #together with a callback fonction
        rospy.Subscriber("/camera/image", Image, callback_follow)
        #define a subscriber on the /scan topic with a import data type before
        #together with a callback fonction
        rospy.Subscriber("/scan", LaserScan, callback_obsta)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass 
