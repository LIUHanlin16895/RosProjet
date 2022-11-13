#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import rospy
import cv_bridge
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
#initialise a new ROS node called 'follow'
rospy.init_node('challenge', anonymous = True)
rate = rospy.Rate(10)

global obstacle # the variable to know if we are close to the obstacle
global linear_x # linear speed
global angular_z # angular speed
global retangle
global threshold # Distance minimale aux obstacles
global direction # direction = False: The  obstacle est right, il faut tourne a left
                 # direction = True: diresction de obstacle est left, il faut tourne a right
global red
global tmp
global front 
global left
global right

front = False
left = False
right = False
#linear_x = 0.22
#linear_x = 0.1
linear_x = 0.1
angular_z = 0.35
red = [0,0]
#red = [0]
tmp = 0
#threshold = rospy.get_param("threshold")
threshold = 0.3
obstacle = False
retangle= False
# direction = False: The  obstacle est right, il faut tourne a left
# direction = True: diresction de obstacle est left, il faut tourne a right
direction = False


#set the color range
lower_red = np.array([160,20,70])
upper_red = np.array([190,255,255])

lower_yellow = np.array([15,0,0])
upper_yellow = np.array([36,255,255])

lower_white = np.array([0,0,240])
upper_white = np.array([255,10,255])

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
    global tmp 
    global front 
    global left
    global right
    
    if tmp ==10:
        data = Twist()
        linear_x = 0
        angular_z = 0
        
        # Sélectionner trois intervalle comme 'avant', 'gauche', 'droit'
        tab_front = msg.ranges[0:25] + msg.ranges[335:360]
    #    tab_front = msg.ranges[0:30] + msg.ranges[330:360]
        tab_leftFront = msg.ranges[10:40]  #############################################################################
        tab_rightFront = msg.ranges[310:350] ##########################################################################
    #    tab_leftFront = msg.ranges[10:120]  #############################################################################
    #    tab_rightFront = msg.ranges[240:350] ##########################################################################

        # Calculer les moyennes des trois tableaus précédent en utilisant la fonciton 'enleverInf(tab)'
        mean_leftFront = enleverInf(tab_leftFront)
        mean_rightFront = enleverInf(tab_rightFront)
        min_front = min(min(tab_front), 1)
        
        if min_front > 0.3:
            linear_x = 0.18
            angular_z = 0
            #if min(msg.ranges[0:60] + msg.ranges[300:360])>1:
                #linear_x = 0
                #angular_z = 0
        else:
            if mean_leftFront > mean_rightFront:
                linear_x = 0.0
                angular_z = 0.8
            elif mean_leftFront <= mean_rightFront:
                linear_x = 0.0
                angular_z = -0.8
            
        data.linear.x = linear_x
        data.angular.z = angular_z 
        pub.publish(data)
        rate.sleep()

    elif tmp ==5:
        obst = 0.25
        tab_front = msg.ranges[0:20] + msg.ranges[340:360]
        tab_left = msg.ranges[20:40]  
        tab_right = msg.ranges[320:340]

        min_front = min(min(tab_front),1)
        min_left = min(min(tab_left),1)
        min_right = min(min(tab_right),1)

        #mean_front = enleverInf(tab_front)
        mean_left = enleverInf(tab_left)
        mean_right = enleverInf(tab_right)

        if min_front <obst :
            front  = True
        else:
            front  = False
            
        if min_left < obst :
            left = True
        else:
            left = False
            
        if min_right < obst  :
            right= True
        else:
            right = False
        
        linear_x = 0.15
        angular_z = 0.2

        
        if left and not front and not right:
            #print("left warn,turn right")
            publisher_obsta(0.01, -angular_z)

        elif right and not front and not left:
            #print("right warn,turn left ")
            publisher_obsta(0.01, angular_z)

        elif front and not right and not left:
            #print("front warn, left and right ok, compare left and right value to turn")
            if mean_left >= mean_right:
            #if min_left >= min_right:    
                #print("turn left")
                publisher_obsta(0.01, angular_z)
            else:
                #print("turn right")
                publisher_obsta(0.01, -angular_z)

        elif left and front and not right:
            #print("left warn, front warn, right ok, turn right")
            #publisher_obsta(0, (-angular_z*2))
            publisher_obsta(0.01, (-angular_z))

        elif right and front and not left:
            #print("left ok, front warn, right warn, turn left")
            #publisher_obsta(0, (angular_z*2))
            publisher_obsta(0.01, (angular_z))

        elif right and left and not front :
            #print("left and right warn, front ok, speed up")
            #publisher_obsta(linear_x*2, 0)
            publisher_obsta(linear_x*0.5, 0)
        
        elif right and front and left:
            #print("left,front,right all warn, turn back")
            publisher_obsta(0.01, angular_z*1.5)

        elif not front and not right and not left:
            #print("No obstacle")
            publisher_obsta(linear_x, 0)

    else:
        # Sélectionner trois intervalle comme 'avant', 'gauche', 'droit'
        tab_front = msg.ranges[0:20] + msg.ranges[340:360]
        #tab_leftFront = msg.ranges[0:40]
        #tab_rightFront = msg.ranges[320:360]
        tab_leftFront = msg.ranges[0:50]
        tab_rightFront = msg.ranges[310:360]
        #tab_leftFront = msg.ranges[20:60]  #############################################################################
        #tab_rightFront = msg.ranges[300:340] ##########################################################################

        # Calculer les moyennes des trois tableaus précédent en utilisant la fonciton 'enleverInf(tab)'
        mean_front = enleverInf(tab_front)
        mean_leftFront = enleverInf(tab_leftFront)
        mean_rightFront = enleverInf(tab_rightFront)
        
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

def publisher_obsta(linear, angular):
    data = Twist() 
    data.linear.x= linear
    data.angular.z = angular
    pub.publish(data)
    rate.sleep() 

#La fonciton qui détect le ligne
def callback_follow(msg):
    global obstacle
    global linear_x 
    global angular_z 
    global retangle
    global red
    global tmp
    
    #creat a bridge 
    cvBridge = cv_bridge.CvBridge ()
    #change the image to opencv and change the RGB to hsv
    cvImage = cvBridge.imgmsg_to_cv2 (msg , desired_encoding='bgr8')
    
    # get the shape of image
    hight, width, deep = cvImage.shape
    
    #determine the center of image
    centre_robot = width/2
    
    hsv = cv2.cvtColor(cvImage,cv2.COLOR_BGR2HSV)
    mask_yellow = cv2.inRange (hsv, lower_yellow, upper_yellow)
    mask_white  = cv2.inRange (hsv,lower_white,upper_white)
    mask_white  = cv2.inRange (hsv,lower_white,upper_white)
    mask_red = cv2.inRange(hsv,lower_red,upper_red)
    
    #Cacher une partie de l'image afin que 
    #la caméra se concentre uniquement sur la piste la plus proche
    search_top = int(4*hight/6)
    mask_yellow[0:search_top, 0:width] = 0
    mask_white[0:search_top, 0:width] = 0
    mask_red[0:int(7*hight/8), 0:width] = 0
    # Afin de passer l'intersection, on cache Nous avons masqué le côté droit de l'image jaune 
    #et le côté gauche de l'image blanche, Mais les deux parties cachées ne sont pas symétriques 
    #et égales,  pour permettre au robot de tourner, on cache plus de parties sur l'image jaune 
    #que sur l'image blanche
    dis = 40
    demi_width = int(width/2)
    mask_yellow[0:hight, (demi_width+dis):width] = 0
    mask_white[0:hight, 0:(demi_width)] = 0
    mask_red[0:hight, 0:(demi_width-5)] = 0
    mask_red[0:hight, (demi_width+5):width] = 0
    
        
    #Calculer les moments
    M_yellow = cv2.moments(mask_yellow)
    M_white  = cv2.moments(mask_white )
    M_red = cv2.moments(mask_red)
    
    if M_yellow[ "m00" ] >0:
        cX_yellow = int (M_yellow[ "m10" ] / M_yellow[ "m00" ] )
        cY_yellow = int (M_yellow[ "m01" ] / M_yellow[ "m00" ] )
    else:
        cX_yellow =5
        cY_yellow = 180
    # draw a circle at center
    cv2.circle(mask_yellow, (cX_yellow ,cY_yellow ), 15, 50,2)

    if M_white [ "m00" ] >0 :          
        cX_white  = int (M_white [ "m10" ] / M_white [ "m00" ] )
        cY_white  = int (M_white [ "m01" ] / M_white [ "m00" ] )
    else:
        cX_white = 315
        cY_white = 180
    # draw a circle at center
    cv2.circle(mask_white, (cX_white ,cY_white ), 15, 150,2)
    
    if M_red["m00"]>0:
        red[0] = 1
    else:
        if red[0] ==1:
            red[1] =1
        if red[0] ==1 and red[1] ==1:
            tmp +=1
            red[0], red[1] = 0,0
    print(tmp)
        
    centre_element = (cX_yellow + cX_white)/2
    #Calculer l'erreur entre centre robot et le centre des deux coleurs
    error = centre_element - centre_robot

    #Afficher les images
    cv2.imshow("Image window1", mask_yellow )
    cv2.imshow("Image window2", mask_white  )
    cv2.imshow("Image window_red", mask_red)
    cv2.waitKey(3)
    
    if tmp != 5:
        #Si il a y un obstacle en avant, on tourne
        if obstacle:
            tourne()     
        #si il y a pas de l'obstacle, on suivre le ligne
        elif not obstacle:
            follow(error)
    
           
def tourne():
    global linear_x 
    global angular_z 
    data = Twist()
    #data.linear.x=linear_x *0.5
    data.linear.x=linear_x  ###################################################################################
    if direction:
        #data.angular.z= - angular_z *2.8# obstacle est left,  tourne vers right
        #data.angular.z= -angular_z *1.5
        data.angular.z= -angular_z *1.5  
        #data.angular.z= -angular_z *1# obstacle est left,  tourne vers right   ###############################################
    else:
        #data.angular.z= angular_z*1 # obstacle est right,  tourne vers left  ##########################################
        data.angular.z= angular_z*1.5
        #data.angular.z= angular_z*1.5
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
    #data.angular.z = -float(error)/150  ########################################################  
    #data.angular.z = -float(error)/100  ######################################################## 
    data.angular.z = -float(error)/60  ########################################################
    data.linear.x=linear_x ###############################################################
    #data.linear.x=linear_x *0.5  ###############################################################
    #data.linear.x=linear_x *0.3
    pub.publish(data)
    rate.sleep()    
           
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