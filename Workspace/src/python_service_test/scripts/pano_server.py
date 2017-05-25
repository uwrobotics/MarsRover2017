##########################################################
# 
# Description:
# Panorama service that sends angles to CAN for the gimbal 
# to turn to. 
# 
# Request:
# int32  doesn't do anything
# 
# Response:
# 1      doesn't do anything
# 
# How to run this service:
# - rosrun python_service_test pano_server.py
# 
# - roscd python_service_test
# - rosparam load config/pano_config.yaml
# 
# - rosservice call /run_pano 1
# 
# Note:
# Angles are calculated by start angle, stop angle, and 
# number of photos to take. These three values are set in 
# config/pano_config.YAML as a list. To change the angles,
# change the corresponding values in the YAML file, save it, 
# and load the params again.
#
##########################################################


#!/usr/bin/env python

from python_service_test.srv import *
import rospy
import struct
from can_msgs.msg import Frame

# service callback
def run_pano_handle(req):
    pan = Frame()
    pan.id = 600
    pan.is_rtr = False
    pan.is_extended = False
    pan.is_error = False
    pan.dlc = 8
    params = rospy.get_param('pano_params') #[start_angle, stop_angle, num_photos]
    step = (params[1] - params[0]) / (params[2] - 1)
    for n in range(params[2]):
        # angle the gimbal will move to
        angle = params[0] + n * step
        pan.data = str(bytearray(struct.pack('i', angle)))
        data = str(struct.unpack('i', pan.data))
        print "Data: " + data
        # move gimbal to desired positions
        pub.publish(pan)
        print "Published", "angle", n + 1
        # sleep for a certain amount of time
        rospy.sleep(2.) 
        # take picture here

    return 1

def pano_server():
    global pub
    pub = rospy.Publisher('/CAN_transmitter', Frame)
    rospy.init_node('pano_server') #name of the service to be run, doesn't really matter
    s = rospy.Service('run_pano', PanoService, run_pano_handle)
#"run_pano" is the thing that you call in a rosservice call (e.g. rosservice call /run_pano 1)
#PanoService is the name of the .srv file in the srv folder
#run_pano_handle is the name of the function to call when the service is called
    print "Ready to run pano script."
    rospy.spin()

if __name__ == "__main__":
    pano_server()

