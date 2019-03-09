#!/usr/bin/env python
from __future__ import division
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose2D
from math import sin, cos, pi

#################################################################
#
#    Odometry Node
#    The node is responsible for calculating and publishing
#    distances and velocities of the robot and wheels
#
#################################################################

#################################################################
#
#   Kinematics info
#   Wheel Base = 18.5 cm
#   Wheel Radius = 3.25 cm
#   PPR ( Motor Side ) = 11 PPR
#   Gear Ratio = 21.3 : 1
#
##################################################################

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

dt = 0.05

def update_odom():
    global rwheel_dist, lwheel_dist, robot_dist, robot_theta, rwheel_vel, lwheel_vel, robot_vel
    global x, y
    global x_c, y_c, th_c

    #Robot Linear Velocity
    robot_dist = (rwheel_dist + lwheel_dist) / 2
    #robot_vel = robot_dist / dt

    #Robot Angular Velocity
    robot_theta = (rwheel_dist - lwheel_dist) / 18.5 # this approximation works (in radians) for small angles
    #robot_omega = robot_theta / dt

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

def rwh_dist(data):
    global rwheel_dist
    rwheel_dist = data.data

def lwh_dist(data):
    global lwheel_dist
    lwheel_dist = data.data

def rwh_vel(data):
    global rwheel_vel
    rwheel_vel = data.data

def lwh_vel(data):
    global lwheel_vel
    lwheel_vel = data.data

#Odometry Listener/Subscribers
def odom_listener():
    global now, then, next
    rospy.init_node('odometry_rob1')
    rospy.loginfo("%s started" % rospy.get_name())

    #Subscribers
    rospy.Subscriber('rwheel_dist_rob1', Float32, rwh_dist)
    rospy.Subscriber('lwheel_dist_rob1', Float32, lwh_dist)
    rospy.Subscriber('rwheel_spd_rob1', Float32, rwh_vel)
    rospy.Subscriber('lwheel_spd_rob1', Float32, lwh_dist)

    #Publishers
    pose = rospy.Publisher('pose_rob1', Pose2D, queue_size=5)
    position = Pose2D()

    while not rospy.is_shutdown():
        update_odom()

        #Update The Position Message
        position.x = x_c
        position.y = y_c
        position.theta = th_c

        pose.publish(position)
        rospy.sleep(dt)

if __name__ == '__main__':
    try:
        odom_listener()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("%s closed" % rospy.get_name())
