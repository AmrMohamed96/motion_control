#!/usr/bin/env python
from __future__ import division
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float32
import RPi.GPIO as GPIO

#################################################################
#
#    The Encoders Node
#    Currently its main function is to interface with encoders
#    and send encoders ticks on topics
#
#################################################################

# Set the GPIO modes
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

#Encoder pins and counters
#Right Motor
encA1 = 23
encB1 = 22
encoder1_ticks = 0

#Left Motor
encA2 = 27
encB2 = 24
encoder2_ticks = 0

def encoders_talker():
    # Initialize the node
    rospy.init_node('encoders_rob1')
    rospy.loginfo("%s started" % rospy.get_name())

    # Initialize encoder publishers
    rate = rospy.Rate(20)
    enc1= rospy.Publisher('enc1_ticks_rob1',Int32, queue_size=20)
    enc2= rospy.Publisher('enc2_ticks_rob1',Int32, queue_size=20)

    while not rospy.is_shutdown():
        # Publishing Calculated Variables in dt
        # Encoder ticks since the beginning
        enc1.publish(encoder1_ticks)
        enc2.publish(encoder2_ticks)
        rate.sleep()

# Updating encoder counters for each interrupt
def do_encoder1(channel1):  #encoder1
	global encoder1_ticks
	if GPIO.input(encB1) == 1:
		encoder1_ticks -= 1
	else:
		encoder1_ticks += 1

def do_encoder2(channel2):  #encoder2
	global encoder2_ticks
	if GPIO.input(encB2) == 1:
		encoder2_ticks += 1
	else:
		encoder2_ticks -= 1

# Encoder 1 GPIO
GPIO.setup (encA1, GPIO.IN, pull_up_down=GPIO.PUD_UP)         # pin input pullup
GPIO.setup (encB1, GPIO.IN, pull_up_down=GPIO.PUD_UP)         # pin input pullup
GPIO.add_event_detect (encA1, GPIO.FALLING, callback=do_encoder1)   # Encoder 1 interrupt

# Encoder 2 GPIO
GPIO.setup (encA2, GPIO.IN, pull_up_down=GPIO.PUD_UP)         # pin input pullup
GPIO.setup (encB2, GPIO.IN, pull_up_down=GPIO.PUD_UP)         # pin input pullup
GPIO.add_event_detect (encA2, GPIO.FALLING, callback=do_encoder2)   # Encoder 2 interrupt

if __name__ == '__main__':
    try:
        encoders_talker()
    except rospy.ROSInterruptException:
        pass
    finally:
        GPIO.cleanup()
        rospy.loginfo("%s closed. GPIO Cleaned" % rospy.get_name())
