#!/usr/bin/env python

from python_service_test.srv import *
import rospy

def run_pano_handle(req):
    print "1st position"
    #move gimbal to first position
    rospy.sleep(2.) #sleep for a certain amount of time
    #take picture here

    print "2nd position"
    #move gimbal to second position
    rospy.sleep(2.)
    #take picture

    print "3rd position"
    #move gimbal to third position 
    rospy.sleep(2.)
    #take picture

    #repeat as needed
    return 1

def pano_server():
    rospy.init_node('pano_server') #name of the service to me run, doesn't really matter
    s = rospy.Service('run_pano', PanoService, run_pano_handle)
#"run_pano" is the thing that you call in a rosservice call (e.g. rosservice call /run_pano 1)
#PanoService is the name of the .srv file in the srv folder
#run_pano_handle is the name of the function to call when the service is called
    print "Ready to run pano script."
    rospy.spin()

if __name__ == "__main__":
    pano_server()
