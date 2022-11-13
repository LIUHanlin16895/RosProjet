#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys, termios, tty
import click
from geometry_msgs.msg import Twist

def publisher():
    
    pub = rospy.Publisher(rospy.get_param("topic_cmd_vel"), Twist, queue_size = 10)
    rospy.init_node('mybot_teleop', anonymous = True)
    rate =rospy.Rate(10)    
    data = Twist()
    
    while not rospy.is_shutdown():
        
        vitesse_lineaire = rospy.get_param("linear_scale")
        vitesse_angulaire = rospy.get_param("angular_scale")
        
        keys = {'\x1b[A':'up', '\x1b[B':'down', '\x1b[C':'right', '\x1b[D':'left', 's':'stop', 'q':'quit'}
        
        mykey = click.getchar()
        if mykey in keys.keys():
            char=keys[mykey]

        if char == 'up':    
            if vitesse_lineaire !=None and vitesse_lineaire !=0:
                data.linear.x = 1*vitesse_lineaire 
            else:
                data.linear.x = 1
        if char == 'down':  
            if vitesse_lineaire !=None and vitesse_lineaire !=0:
                data.linear.x = -1*vitesse_lineaire 
            else:
                data.linear.x = -1

        if char == 'left': 
            if vitesse_angulaire !=None and vitesse_angulaire !=0:
                data.angular.z= 1*vitesse_angulaire 
            else:
                data.angular.z= 1
            
        if char == 'right': 
            if vitesse_angulaire !=None and vitesse_angulaire !=0:
                data.angular.z= -1*vitesse_angulaire 
            else:
                data.angular.z= -1

        if char == "quit":  
            data.linear.x = 0
            data.angular.x = 0
            sys.exit()
        
        pub.publish(data)
        rate.sleep()
        data.linear.x = 0
        data.angular.z = 0
        pub.publish(data)
    
  

if __name__ == '__main__':

    try:   
        publisher()

    except rospy.ROSInterruptException:
        pass

