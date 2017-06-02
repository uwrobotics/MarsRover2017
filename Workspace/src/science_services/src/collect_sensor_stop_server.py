#!/usr/bin/env python

##############################################################
# 
# @file      collect_sensor_stop_server.py
# @author    Ivy Xing
# 
# Description:
# Service that sends a stop signal to CAN to stop collecting
# sensor info.
# 
# Request:
# int32  doesn't do anything
# 
# Response:
# 1      doesn't do anything
# 
# How to run this service:
# - rosrun science_services collect_sensor_stop_server.py
# 
# - rosservice call /stop_collect_sensor 1
#
##############################################################


from science_services.srv import collect_sensor_stop_service
import rospy
import struct
from can_msgs.msg import Frame

# service callback
def stop_collect_sensor_handle(req):
    frame = Frame()
    frame.id = 712
    frame.is_rtr = False
    frame.is_extended = False
    frame.is_error = False
    frame.dlc = 8
    frame.data = str(bytearray(struct.pack('i', 1)))
    pub.publish(frame)
    print "Stop sent"
    return 1

def collect_sensor_stop_server():
    global pub
    pub = rospy.Publisher('/CAN_transmitter', Frame)
    rospy.init_node('collect_sensor_stop_server') #name of the service to be run, doesn't really matter
    s = rospy.Service('stop_collect_sensor', collect_sensor_stop_service, stop_collect_sensor_handle)
#"stop_collect_sensor" is the thing that you call in a rosservice call (e.g. rosservice call /stop_collect_sensor 1)
#collect_sensor_stop_service is the name of the .srv file in the srv folder
#stop_collect_sensor_handle is the name of the function to call when the service is called
    print "Ready to run stop_collect_sensor script."
    rospy.spin()

if __name__ == "__main__":
    collect_sensor_stop_server()

