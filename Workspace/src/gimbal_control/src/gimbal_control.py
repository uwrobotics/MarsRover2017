#!/usr/bin/env python

###############################################################
# 
# @file      gimbal_control.py
# @author    Ivy Xing / Jerry Li
# 
# Description:
# Gimbal control node that sends angles to CAN according to
# gimbal ros topic
# 
# How to run this node:
# - rosrun gimbal_control gimbal_control.py
# 
# Note:
# - DO NOT MOVE JOYSTICK WHEN PRESSING GIMBAL CONTROL BUTTONS!
#   The angles will change rapidly
# - Angles change 15 degrees every time corresponding button
#   is pressed.
# - Pan: 0 to 360. Angles wrap around.
# - Tilt: -45 to 45. 
# - Send -999 as an angle if you don't want it to move
#
###############################################################


import rospy
import roslib
import struct
import sys

from can_msgs.msg import Frame
from std_msgs.msg import Int32MultiArray

class GimbalController:
	# Constructor
	def __init__(self, sub_topic='/gimbal_cmd', pub_topic='/CAN_transmitter'):
		self._node = rospy.init_node('gimbal_control', anonymous=True)
		self._joySub = rospy.Subscriber(sub_topic, Int32MultiArray, self.gimbalCallback)
		self._canPub = rospy.Publisher(pub_topic, Frame, queue_size=20)
		self._pan = 90
		self._tilt = 0
		self._lastpan = 90
		self._lasttilt = 0
		self._id = 600 # CAN id
		self.startGimbal();

	# Set gimbal to the start position. Only called once at the start.
	def startGimbal(self):
		print 'Initializing Gimbal'
		self.sendAngle()

	# Joystick ROS callback
	def gimbalCallback(self, gimbalMsg):
		if gimbalMsg.data[0] != -999: #invalid angle
			self._pan = gimbalMsg.data[0]

		if gimbalMsg.data[1] != -999:
			self._tilt = gimbalMsg.data[1]
		print 'Sending to Gimbal: Pan: %d Tilt: %d' % (self._pan, self._tilt)
		self.sendAngle()

	# Convert angles to CAN frame msg type
	def angleToFrame(self):
		frame = Frame()
		frame.id = self._id
		frame.is_rtr = False
		frame.is_extended = False
		frame.is_error = False
		frame.dlc = 8
		data = bytearray(struct.pack('i', self._pan))
		data.extend(bytearray(struct.pack('i', self._tilt)))
		frame.data = str(data)
		return frame

	# Publish angles to SocketCAN
	def sendAngle(self):
		canFrame = self.angleToFrame()
		self._canPub.publish(canFrame)


if __name__ == '__main__':
	controller = GimbalController()
	rospy.spin()
