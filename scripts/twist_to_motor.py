#!/usr/bin/env python
import rospy
import roslib
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

"""
    *** Twist to Motors Nodes ***
    The node takes twist messages for the robot navigation
    and converts them to wheel velocities
"""

# Wheel Base Length (cm)
w = 18.5 

# Twist message Variables
dx = 0
dy = 0
dr = 0

r_target = 0
l_target = 0

# The function that converts the twist message into wheel velocities
def twistToVel():
    global r_target, l_target

    # dx = (l + r) / 2
    # dr = (r - l) / w
    r_target = 1.0 * dx + dr * w / 2
    l_target = 1.0 * dx - dr * w / 2

# Call back function to store the message
def twistCallback(data):
    global dx, dy, dr
    dx = data.linear.x
    dy = data.linear.y
    dr = data.angular.z

def twister():
    # Initializing the Node
    rospy.init_node('twister_rob1')
    rospy.loginfo("%s started" % rospy.get_name())

    # Subscriber to the Twist Message
    rospy.Subscriber('cmd_vel', Twist, twistCallback)

    pub_rmotor = rospy.Publisher('rwheel_vtarget_rob1', Float32, queue_size=20)
    pub_lmotor = rospy.Publisher('lwheel_vtarget_rob1', Float32, queue_size=20)

    while not rospy.is_shutdown():
        twistToVel()
        pub_rmotor.publish(r_target)
        pub_lmotor.publish(l_target)

if __name__ == '__main__':
    try:
        twister()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("%s closed. All PID Vars are now reset" % rospy.get_name())
