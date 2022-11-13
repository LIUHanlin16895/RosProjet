#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys, termios, tty
import click
import time
from geometry_msgs.msg import Twist

# Arrow keys codes
keys = {'\x1b[A':'up', '\x1b[B':'down', '\x1b[C':'right', '\x1b[D':'left', 's':'stop', 'q':'quit'}

l=rospy.get_param("/linear_scale")
a=rospy.get_param("/angular_scale")
c=rospy.get_param("topic_choice")

def talker():
    pub = rospy.Publisher(c, Twist, queue_size=10)
    rospy.init_node('mybot_teleop', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        vel_msg = Twist()
        # Get character from console
        mykey = click.getchar()
        if mykey in keys.keys():
            char=keys[mykey]

        if char == 'up':    # UP key
            vel_msg.linear.x = 1*l
        if char == 'down':  # DOWN key
            vel_msg.linear.x = -1*l
        if char == 'left':  # RIGHT key
            vel_msg.angular.z = 1*a
        if char == 'right': # LEFT
            vel_msg.angular.z = -1*a
        if char == "quit":  # QUIT
            break
        pub.publish(vel_msg)
        time.sleep(0.5)
        pub.publish(Twist())
        rate.sleep() 

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
