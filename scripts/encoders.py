#!/usr/bin/env python
from __future__ import division
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float32
import math
import time
import RPi.GPIO as GPIO

"""
     *** The Encoders Node ***
     Interfaces the magnetic encoders with the RPi3 B/B+
     Calculates Encoder Ticks / Wheel Speeds / Wheel Distances

     *** Kinematics info ***
     Wheel Base = 18.5 cm
     Wheel Radius = 3.25 cm
     PPR ( Motor Side ) = 11 PPR
     Gear Ratio = 21.3 : 1
"""

# Set the GPIO modes
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

""" Encoders Pins and Counter Variables """
# Right Motor
encA1 = 23 #22
encB1 = 22 #27
encoder1_ticks = 0
encoder1_ticks_calc = 0
encoder1_ticks_prev = 0
r_ticks = 0

# Left Motor
encA2 = 27 #11
encB2 = 24 #9
encoder2_ticks = 0
encoder2_ticks_calc = 0
encoder2_ticks_prev = 0
l_ticks = 0

# Distance Variables
rwheel_dist = 0
lwheel_dist = 0

# Speed Variables
rwheel_spd = 0
lwheel_spd = 0

# Delta Time for Calculations
dt = 0.05

# dist_const is used to facilitate dist/speed calculations respectively = (distance per rev) / (PPR * GR)
dist_const = ( math.pi * 6.5 ) / ( 11 * 21.3 ) # Used to get distance covered in CM

def encoders_talker():
    global encoder1_ticks, encoder2_ticks
    global rwheel_dist, lwheel_dist
    global rwheel_spd, lwheel_spd
    global encoder1_ticks_prev, encoder2_ticks_prev
    global r_ticks, l_ticks

    # Initialize the node
    rospy.init_node('encoders_rob1')
    rospy.loginfo("%s started" % rospy.get_name())

    # Initialize encoder publishers
    enc1= rospy.Publisher('enc1_ticks_rob1',Int32, queue_size=10)
    enc2= rospy.Publisher('enc2_ticks_rob1',Int32, queue_size=10)

    #Speed/Position Publishers
    rwheelSpeed = rospy.Publisher('rwheel_spd_rob1', Float32, queue_size=5)
    lwheelSpeed = rospy.Publisher('lwheel_spd_rob1', Float32, queue_size=5)

    rwheelDist = rospy.Publisher('rwheel_dist_rob1', Float32, queue_size=5)
    lwheelDist = rospy.Publisher('lwheel_dist_rob1', Float32, queue_size=5)

    while not rospy.is_shutdown():
        # Saving current ticks because this should be a non-reenterant function
        encoder1_ticks_calc = encoder1_ticks
        encoder2_ticks_calc = encoder2_ticks

        # Ticks recorded in dt
        r_ticks = encoder1_ticks_calc - encoder1_ticks_prev
        l_ticks = encoder2_ticks_calc - encoder2_ticks_prev

        # Saving current ticks as previous ticks
        encoder1_ticks_prev = encoder1_ticks_calc
        encoder2_ticks_prev = encoder2_ticks_calc

        # Distance covered in dt
        rwheel_dist =  ( r_ticks * dist_const )
        lwheel_dist =  ( l_ticks * dist_const )

        # Speed in cm/s
        rwheel_spd = rwheel_dist / dt
        lwheel_spd = lwheel_dist / dt

        # Publishing Calculated Variables in dt
        # Encoder ticks since the beginning
        enc1.publish(encoder1_ticks)
        enc2.publish(encoder2_ticks)

        # Wheel Velocities
        rwheelSpeed.publish(rwheel_spd)
        lwheelSpeed.publish(lwheel_spd)

        # Distances covered by wheels since the beginning
        rwheelDist.publish(rwheel_dist)
        lwheelDist.publish(lwheel_dist)

        # Wait dt
        time.sleep(dt)

# Updating encoder counters for each interrupt
# Encoder 1
def do_encoder1(channel1):  
	global encoder1_ticks
	if GPIO.input(encB1) == 1:
		encoder1_ticks -= 1
	else:
		encoder1_ticks += 1

# Encoder 2
def do_encoder2(channel2): 
	global encoder2_ticks
	if GPIO.input(encB2) == 1:
		encoder2_ticks += 1
	else:
		encoder2_ticks -= 1

# Encoder 1 GPIO
GPIO.setup (encA1, GPIO.IN, pull_up_down=GPIO.PUD_UP)                 # pin input pullup
GPIO.setup (encB1, GPIO.IN, pull_up_down=GPIO.PUD_UP)                 # pin input pullup
GPIO.add_event_detect (encA1, GPIO.FALLING, callback=do_encoder1)     # Encoder 1 interrupt

# Encoder 2 GPIO
GPIO.setup (encA2, GPIO.IN, pull_up_down=GPIO.PUD_UP)                 # pin input pullup
GPIO.setup (encB2, GPIO.IN, pull_up_down=GPIO.PUD_UP)                 # pin input pullup
GPIO.add_event_detect (encA2, GPIO.FALLING, callback=do_encoder2)     # Encoder 2 interrupt

if __name__ == '__main__':
    try:
        encoders_talker()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()
        rospy.loginfo("%s closed. GPIO Cleaned" % rospy.get_name())
