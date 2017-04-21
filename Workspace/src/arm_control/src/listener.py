#!/usr/bin/env python
import rospy
import struct

from can_msgs.msg import Frame

def callback(data):
    unpacked_cmd = struct.unpack('f', data.data[0:4])
    cmd = unpacked_cmd[0]
    unpacked_idx = struct.unpack('B', data.data[4])
    idx = unpacked_idx[0]
    rospy.loginfo("Received cmd %f from index %u" % (float(cmd), idx))

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("/arm_control", Frame, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
