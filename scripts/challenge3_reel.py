#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy as np
import rospy
import cv_bridge
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
#initialise a new ROS node called 'navigation'
rospy.init_node('navigation', anonymous = True)
rate = rospy.Rate(10)

global front 
global left
global right
global sorti
global threshold

front = False
left = False
right = False
sorti = False
threshold = 0.25

def enleverInf(tab):
    res = [i for i in tab if i <0.35]  
    return np.mean(res) if res else 10

def callback(msg):
    global front 
    global left
    global right
    global sorti
    global threshold

    tab_front = msg.ranges[0:20] + msg.ranges[340:360]
    tab_left = msg.ranges[20:40]  
    tab_right = msg.ranges[320:340]

    min_front = min(min(tab_front),1)
    min_left = min(min(tab_left),1)
    min_right = min(min(tab_right),1)

    #mean_front = enleverInf(tab_front)
    mean_left = enleverInf(tab_left)
    mean_right = enleverInf(tab_right)

    if min_front < threshold :
        front  = True
    else:
        front  = False
        
    if min_left < threshold :
        left = True
    else:
        left = False
        
    if min_right < threshold :
        right= True
    else:
        right = False
    
    linear_x = 0.15
    angular_z = 0.2

    if min(msg.ranges[0:360])> 0.5:
        print('stop')
        publisher_obsta(0, 0)
    else:
        if left and not front and not right:
            print("left warn,turn right")
            publisher_obsta(0.01, -angular_z)

        elif right and not front and not left:
            print("right warn,turn left ")
            publisher_obsta(0.01, angular_z)

        elif front and not right and not left:
            print("front warn, left and right ok, compare left and right value to turn")
            if mean_left >= mean_right:
            #if min_left >= min_right:    
                print("turn left")
                publisher_obsta(0.01, angular_z)
            else:
                print("turn right")
                publisher_obsta(0.01, -angular_z)

        elif left and front and not right:
            print("left warn, front warn, right ok, turn right")
            #publisher_obsta(0, (-angular_z*2))
            publisher_obsta(0.01, (-angular_z))

        elif right and front and not left:
            print("left ok, front warn, right warn, turn left")
            #publisher_obsta(0, (angular_z*2))
            publisher_obsta(0.01, (angular_z))

        elif right and left and not front :
            print("left and right warn, front ok, speed up")
            #publisher_obsta(linear_x*2, 0)
            publisher_obsta(linear_x*0.5, 0)
        
        elif right and front and left:
            print("left,front,right all warn, turn back")
            publisher_obsta(0.01, angular_z*1.5)

        elif not front and not right and not left:
            print("No obstacle")
            publisher_obsta(linear_x, 0)

def publisher_obsta(linear, angular):
    data = Twist() 
    data.linear.x= linear
    data.angular.z = angular
    pub.publish(data)
    rate.sleep()   


if __name__ == '__main__':
    try:
        #create a publiser, publise a topic named 'cmd_vel',
        #type of message is Twist. The length of queue is 10
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        #define a subscriber on the /camera/image topic with a import data type before
        #together with a callback fonction
        #rospy.Subscriber("/camera/image", Image, callback_follow);
        #define a subscriber on the /scan topic with a import data type before
        #together with a callback fonction
        rospy.Subscriber("/scan", LaserScan, callback)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass