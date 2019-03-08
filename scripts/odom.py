#!/usr/bin/env python
from __future__ import division
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from math import sin, cos, pi

#################################################################
#
#    Odometry Node
#    The node is responsible for calculating and publishing
#    distances and velocities of the robot and wheels
#
#################################################################

######################################Creating your ow###########################
#
#   Kinematics info
#   Wheel Base = 18.5 cm
#   Wheel Radius = 3.25 cm
#   PPR ( Motor Side ) = 11 PPR
#   Gear Ratio = 21.3 : 1
#
##################################################################

#Time Variables - Used for Time Calculations
dt = 0.05

#Variables for encoder ticks (Enc1: Right Motor - Enc2: Left Motor)
encoder1 = 0
encoder1_prev = 0
r_ticks = 0

encoder2 = 0
encoder2_prev = 0
l_ticks = 0

#Robot Odometry Variables
rwheel_dist = 0
lwheel_dist = 0

rwheel_vel = 0
lwheel_vel = 0

robot_dist = 0
robot_vel = 0
robot_theta = 0
robot_omega = 0

#Distances travelled in x, y
x = 0
y = 0

#Current Position assuming the robot starts from zero
x_c = 0
y_c = 0
th_c = 0

def update_odom():
    global encoder1, encoder1_prev, r_ticks, encoder2, encoder2_prev, l_ticks
    global rwheel_dist, lwheel_dist, robot_dist, robot_theta, rwheel_vel, lwheel_vel, robot_vel
    global x, y
    global x_c, y_c, th_c

    #Calculate ticks that occured in dt
    r_ticks = encoder1 - encoder1_prev
    l_ticks = encoder2 - encoder2_prev
    encoder1_prev = encoder1
    encoder2_prev = encoder2

    rwheel_dist = (r_ticks * 2 * pi * 3.25) / (11 * 21.3 )
    lwheel_dist = (l_ticks * 2 * pi * 3.25) / (11 * 21.3 )

    rwheel_vel = rwheel_dist / dt
    lwheel_vel = lwheel_dist / dt

    #Robot Linear Velocity
    robot_dist = (rwheel_dist + lwheel_dist) / 2
    robot_vel = robot_dist / dt

    #Robot Angular Velocity
    robot_theta = (rwheel_dist - lwheel_dist) / 18.5 # this approximation works (in radians) for small angles
    robot_omega = robot_theta / dt

    if (robot_dist != 0):
        #Update Distances covered in x, y
        x = robot_dist * cos(robot_theta)
        y = robot_dist * -sin(robot_theta)

        x_c = x_c + ( cos( th_c ) * x - sin( th_c ) * y )
        y_c = y_c + ( sin( th_c ) * x + cos( th_c ) * y )

    if( robot_theta != 0):
        th_c = th_c + robot_theta

#Call Back Functions for Subscribers
def enc1_callback(data):
    global encoder1
    encoder1 = data.data

def enc2_callback(data):
    global encoder2
    encoder2 = data.data

#Odometry Listener/Subscribers
def odom_listener():
    global now, then, next
    rospy.init_node('odometry_rob1')
    rospy.loginfo("%s started" % rospy.get_name())

    #Initialization for time variables
    then = rospy.get_time() + dt

    #Subscribers
    rospy.Subscriber('enc1_ticks_rob1', Int32, enc1_callback)
    rospy.Subscriber('enc2_ticks_rob1', Int32, enc2_callback)

    #Publishers
    dist_r = rospy.Publisher('rwheel_dist_rob1', Float32, queue_size = 20)
    dist_l = rospy.Publisher('lwheel_dist_rob1', Float32, queue_size = 20)
    vel_r = rospy.Publisher('rwheel_spd_rob1', Float32, queue_size = 20)
    vel_l = rospy.Publisher('lwheel_spd_rob1', Float32, queue_size = 20)

    X = rospy.Publisher('x_rob1', Float32, queue_size = 20)
    Y = rospy.Publisher('y_rob1', Float32 , queue_size = 20)
    Th = rospy.Publisher('th_rob1', Float32, queue_size = 20)

    while not rospy.is_shutdown():
        update_odom()
        dist_r.publish(rwheel_dist)
        dist_l.publish(lwheel_dist)
        vel_r.publish(rwheel_vel)
        vel_l.publish(lwheel_vel)

        X.publish(x_c)
        Y.publish(y_c)
        Th.publish(th_c)

        rospy.sleep(dt)

if __name__ == '__main__':
    try:
        odom_listener()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("%s closed" % rospy.get_name())
