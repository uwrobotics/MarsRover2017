#!/usr/bin/env python

###############################################################
# 
# @file      gimbal_control.py
# @author    Ivy Xing
# 
# Description:
# Gimbal control node that sends angles to CAN according to
# buttons on the joystick.
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
#
###############################################################


import rospy
import roslib
import struct
import sys

from sensor_msgs.msg import Joy
from can_msgs.msg import Frame

class GimbalController:
	# Constructor
	def __init__(self, sub_topic='/joy', pub_topic='/CAN_transmitter'):
		self._node = rospy.init_node('gimbal_control', anonymous=True)
		self._joySub = rospy.Subscriber(sub_topic, Joy, self.joyCallback)
		self._canPub = rospy.Publisher(pub_topic, Frame, queue_size=20)
		self._started = False
		self._pan = 90
		self._tilt = 0
		self._id = 600 # CAN id
		self._step = 15 # the angle to turn for each move

	# Set gimbal to the start position. Only called once at the start.
	def startGimbal(self):
		print 'starting'
		self.sendAngle(self._pan, self._tilt, self._id)

	# Joystick ROS callback
	def joyCallback(self, joyMsg):
		# Start the gimbal the first time this is called
		if not self._started:
			self.startGimbal();
			self._started = True;
			return;

		buttons = list(joyMsg.buttons)
		# up: 2, down: 1, left: 3, right: 4
		up = buttons[2]
		down = buttons[1]
		left = buttons[3]
		right = buttons[4]

		# calculate the next pan and tilt if any of the four buttons is pressed
		if up + down + left + right != 0:
			self._pan += self._step * (right - left)
			# wrap around 360
			self._pan %= 360

			self._tilt += self._step * (up - down)
			# limit tilt to max range
			if self._tilt > 45:
				self._tilt = 45
			elif self._tilt < -45:
				self._tilt = -45

			self.sendAngle(self._pan, self._tilt, self._id)

	# Convert angles to CAN frame msg type
	def angleToFrame(self, pan, tilt, id):
		frame = Frame()
		frame.id = id
		frame.is_rtr = False
		frame.is_extended = False
		frame.is_error = False
		frame.dlc = 8
		data = bytearray(struct.pack('i', pan))
		data.extend(bytearray(struct.pack('i', tilt)))
		frame.data = str(data)
		return frame

	# Publish angles to SocketCAN
	def sendAngle(self, pan, tilt, id):
		canFrame = self.angleToFrame(pan, tilt, id)
		self._canPub.publish(canFrame)


if __name__ == '__main__':
	controller = GimbalController()
	rospy.spin()
