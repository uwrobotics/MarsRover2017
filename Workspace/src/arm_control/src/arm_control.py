#!/usr/bin/env python

import serial
import rospy
import roslib
import struct
import sys

from sensor_msgs.msg import Joy
from can_msgs.msg import Frame

class ArmController:
	# Constructor
	def __init__(self, sub_topic='/joy', pub_topic='/CAN_transmitter'):
		self._node = rospy.init_node('arm_control', anonymous=True)
		self._joySub = rospy.Subscriber(sub_topic, Joy, self.joyCallback)
		self._canPub = rospy.Publisher(pub_topic, Frame, queue_size=20)
		self.__sendZeros = True

	# Joystick ROS callback
	def joyCallback(self, joyMsg):
		# On Logitech Attack3 joystick, x-axis is axis 0, y-axis is axis 1, trigger is button 0
		axes = list(joyMsg.axes)
		buttons = list(joyMsg.buttons)
		switchOn = buttons[0] # trigger is deadman's switch

		# Cap values to max. speed
		if switchOn:
			# for i in range(len(axes)):
			# 	# cap duty cycle values to certain value
			# 	axes[i] *= 0.5
			# 	self.sendCommand(axes[i], i)
			axes[0] *= 0.5
			self.sendCommand(axes[0], 0)
			self.__sendZeros = True

		# Send 0s only once if deadman's switch released
		elif self.__sendZeros:
			# for i in range(len(axes)):
			# 	self.sendCommand(0, i)
			self.sendCommand(0, 0)
			self.__sendZeros = False

	# Convert cmd to CAN frame msg type
	def cmdToFrame(self, cmd, idx):
		frame = Frame()
		frame.id = 5
		frame.is_rtr = False
		frame.is_extended = False
		frame.is_error = False
		frame.dlc = 5
		data = bytearray(struct.pack('f', cmd))
		data.append(struct.pack('B', idx))
		frame.data = str(data)
		return frame

	# Publish command to SocketCAN
	def sendCommand(self, cmd, idx):
		canFrame = self.cmdToFrame(cmd, idx)
		self._canPub.publish(canFrame)


if __name__ == '__main__':
	controller = ArmController()
	rospy.spin()
