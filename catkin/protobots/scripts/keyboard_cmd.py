#!/usr/bin/env python

import rospy
import pygame
from geometry_msgs.msg import Twist
import time

WIDTH =160
HEIGHT = 120

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()


def talker():
    rospy.init_node('cmdvel_serial', anonymous=True)    
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=20)
    rate = rospy.Rate(5) # 10hz
    
    while not rospy.is_shutdown():
        varLeft = 0
        varRight = 0
        varUp = 0
        varDown = 0
        mod =1
        clock.tick(5)
        keys = pygame.key.get_pressed()
        time.sleep(0.1)
        
        if pygame.key.get_mods() & pygame.KMOD_SHIFT:
                print("shift")
                mod = 2
    
        if keys[pygame.K_a]:

                print("Left !")
                varLeft = 0.7
                
        if keys[pygame.K_d]:

                print("Right !")
                varRight = 0.7
                
        if keys[pygame.K_w]:

                print("Up !")
                varUp = 0.2
                
        if keys[pygame.K_s]:

                print("Down !")
                varDown = 0.2
                
        if keys[pygame.K_SPACE]:
                print("Stop !")
                varDown =0
                varUp =0
                varRight =0
                varLeft=0

             
        if keys[pygame.K_ESCAPE]:
                break

        vel_msg = Twist()

        vel_msg.linear.x = (varUp-varDown)*mod
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = (varLeft-varRight)*mod
        
        pub.publish(vel_msg)
        pygame.event.pump()  # process event queue
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass



