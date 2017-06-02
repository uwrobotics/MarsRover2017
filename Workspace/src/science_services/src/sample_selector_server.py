#!/usr/bin/env python

##############################################################
# 
# @file      sample_selector_server.py
# @author    Ivy Xing
# 
# Description:
# Service that takes in and sends a float to CAN to move the 
# motor for sample selector.
# 
# Request:
# float32  input    duty cycle for the motor
# 
# Response:
# 1        doesn't do anything
# 
# How to run this service:
# - rosrun science_services sample_selector_server.py
# 
# - rosservice call /run_sample_selector [float]
#
##############################################################


from science_services.srv import sample_selector_service
import rospy
import struct
from can_msgs.msg import Frame

# service callback
def sample_selector_handle(req):
    frame = Frame()
    frame.id = 701
    frame.is_rtr = False
    frame.is_extended = False
    frame.is_error = False
    frame.dlc = 8
    frame.data = str(bytearray(struct.pack('f', req.input)))
    data = str(struct.unpack('f', frame.data))
    pub.publish(frame)
    print "Data sent:", data
    return 1

def sample_selector_server():
    global pub
    pub = rospy.Publisher('/CAN_transmitter', Frame)
    rospy.init_node('sample_selector_server') #name of the service to be run, doesn't really matter
    s = rospy.Service('run_sample_selector', sample_selector_service, sample_selector_handle)
#"run_sample_selector" is the thing that you call in a rosservice call (e.g. rosservice call /run_sample_selector 1.0)
#sample_selector_service is the name of the .srv file in the srv folder
#sample_selector_handle is the name of the function to call when the service is called
    print "Ready to run run_sample_selector script."
    rospy.spin()

if __name__ == "__main__":
    sample_selector_server()

