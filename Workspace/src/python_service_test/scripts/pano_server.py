#!/usr/bin/env python

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
# config/pano_config.YAML as a list. To change the angles
# or the time to wait after angles for each step is sent,
# change the corresponding values in the YAML file, save it, 
# and load the params again.
#
##########################################################


from python_service_test.srv import *
import rospy
import struct
from can_msgs.msg import Frame
from std_msgs.msg import Int32MultiArray
from  std_srvs.srv import Empty
import os
from os.path import expanduser

# service callback
def run_pano_handle(req):
    params = rospy.get_param('pano_params') #[start_angle, stop_angle, num_photos]
    step = (params[1] - params[0]) / (params[2] - 1)
    gimbal_msg = Int32MultiArray()
    msg_array = [0, -999]#invalid tilt angle to signify to keep at the same angle

    for n in range(params[2]):
        # angle the gimbal will move to
        angle = params[0] + n * step
        msg_array[0] = angle
        gimbal_msg.data = msg_array

        # move gimbal to desired positions
        pub.publish(gimbal_msg)
        print "Published", "angle", n + 1

        # sleep for a certain amount of time
        wait_time = rospy.get_param('wait_time')
        rospy.sleep(wait_time) 
        # take picture here (manually)
        rospy.wait_for_service('image_saver/save')
        try:
            capture = rospy.ServiceProxy('image_saver/save', Empty)
            capture()
        except rospy.ServiceException, e:
            print "Error calling image_saver/save"
            return -1

    print "Done Panorama"
    return 1

def pano_server():
    global pub
    pub = rospy.Publisher('/gimbal_cmd', Int32MultiArray)
    rospy.init_node('pano_server') #name of the service to be run, doesn't really matter
    s = rospy.Service('run_pano', PanoService, run_pano_handle)
    #"run_pano" is the thing that you call in a rosservice call (e.g. rosservice call /run_pano 1)
    #PanoService is the name of the .srv file in the srv folder
    #run_pano_handle is the name of the function to call when the service is called

    #images will be saved here
    directory = expanduser("~") + '/pano_images'
    if not os.path.exists(directory):
        os.makedirs(directory)
    print directory

    print "Ready to run pano script."
    rospy.spin()

if __name__ == "__main__":
    pano_server()

