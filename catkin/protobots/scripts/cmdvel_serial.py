#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import serial
import math
from math import sin, cos, pi
import time
import tf
from nav_msgs.msg import Odometry

ser = serial.Serial("/dev/ttyArdM",115200,timeout = 0.2)
time.sleep(1)  #Starting arsuino
FACTORX = 100 * 1.15  #scaling motor driver commands to m/s
FACTORTH = 100 * 0.13

def callback(data):
    vel_msg = Twist()
    if data.linear.z == -1 :
        ser.write("reset\n\r")

    vel_msg.linear.x = data.linear.x
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = data.angular.z
    #rospy.loginfo(rospy.get_caller_id() + "I heard :")
    #rospy.loginfo(vel_msg)

    ser.flushOutput()
    time.sleep(0.05)
    ser.write("move {0} {1}\n\r".format(int(vel_msg.linear.x*FACTORX),int(vel_msg.angular.z*FACTORTH)).encode())
    


def talker():
    rospy.init_node('cmdvel_serial', anonymous=True)    
    pub = rospy.Publisher('odom', Odometry, queue_size=20)
    odom_broadcaster = tf.TransformBroadcaster()
    laser_frame_broadcaster = tf.TransformBroadcaster()
    rospy.Subscriber("cmd_vel", Twist, callback)
    rate = rospy.Rate(10) # 10hz

    
    px =0.0
    py =0.0
    th =0.0
    lastPx=0.0
    lastPy=0.0
    lastTh=0.0

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    while not rospy.is_shutdown():
        odom = ser.readline()
        rospy.loginfo(odom)
        if  odom.count(',') ==2:
            a,b,c = odom.split(",")
            px = float(a)/100    #cm to m
            py = float(b)/100
            th = float(c)*pi/180  #deg to rad
            #rospy.loginfo("rec@!!!!!!!!!!")

        current_time = rospy.Time.now()
        dt = (current_time - last_time).to_sec()
        dx = (px - lastPx)/dt
        dy = (py - lastPy)/dt
        # if the theta is near zero degree, we should correct the jump in the diff 
        if th-lastTh < -pi:   #robot cannot rotate at this speed, so it is a jump near zero degree
            vth = (th - lastTh + 2*pi)/dt
        elif th-lastTh > pi:
            vth = (th - lastTh - 2*pi)/dt
        else:
            vth = (th - lastTh)/dt

        #This part can be simplified for a differencial robot (TO DO)
        vx = dx * cos(th) + dy * sin(th)
        vy = -(dx * sin(th)) + dy * cos(th)  # should be 0 for the diff robot !!!!
        
        #rospy.loginfo(th)
        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

        odom_broadcaster.sendTransform(
            (px, py, 0.),
            odom_quat,
            current_time,
            "base_link",
            "odom")

   #     laser_frame_broadcaster.sendTransform(
    #        (px, py, 0.),
    #        odom_quat,
    #        current_time,
   #         "laser",
   #         "base_link")
        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        # set the position
        odom.pose.pose = Pose(Point(px, py, 0.), Quaternion(*odom_quat))
        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

        
        #ser.flushInput()
        #hello_str = "hello world %s" % rospy.get_time()
        #rospy.loginfo(hello_str)
        ser.flushInput()
        pub.publish(odom)
        lastPx=px
        lastPy=py
        lastTh=th
        last_time = current_time
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass



