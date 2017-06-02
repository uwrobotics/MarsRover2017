#!/usr/bin/env python

##############################################################
# 
# @file      drill_elevator_server.py
# @author    Ivy Xing
# 
# Description:
# Service that takes in two floats, combines them in one CAN
# message, and sends it to CAN to move the motors for drill
# and elevator.
# 
# Request:
# float32    drill       duty cycle for the drill motor
# float32    elevator    duty cycle for the elevator motor
# 
# Response:
# 1      doesn't do anything
# 
# How to run this service:
# - rosrun science_services drill_elevator_server.py
# 
# - rosservice call /run_drill_elevator [float] [float]
#
##############################################################


from science_services.srv import drill_elevator_service
import rospy
import struct
from can_msgs.msg import Frame

# service callback
def drill_elevator_handle(req):
    frame = Frame()
    frame.id = 700
    frame.is_rtr = False
    frame.is_extended = False
    frame.is_error = False
    frame.dlc = 8
    data = bytearray(struct.pack('f', req.drill))
    data.extend(bytearray(struct.pack('f', req.elevator)))
    frame.data = str(data)
    pub.publish(frame)
    print "Data sent"
    return 1

def drill_elevator_server():
    global pub
    pub = rospy.Publisher('/CAN_transmitter', Frame)
    rospy.init_node('drill_elevator_server') #name of the service to be run, doesn't really matter
    s = rospy.Service('run_drill_elevator', drill_elevator_service, drill_elevator_handle)
#"run_drill_elevator" is the thing that you call in a rosservice call (e.g. rosservice call /run_drill_elevator 1)
#drill_elevator_service is the name of the .srv file in the srv folder
#drill_elevator_handle is the name of the function to call when the service is called
    print "Ready to run run_drill_elevator script."
    rospy.spin()

if __name__ == "__main__":
    drill_elevator_server()

