#!/usr/bin/env python

###############################################################
# 
# @file      gimbal_control.py
# @author    Jerry Li
# 

import rospy
import roslib
import struct
import sys
from can_msgs.msg import Frame
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger

class DrillController:

    def start(self, req):
        self._started = not self._started
        frame = Frame()
        frame.id = self._start_id
        frame.is_rtr = False
        frame.is_error = False
        frame.is_extended = False
        frame.dlc = 8
        data = bytearray(struct.pack('i', 1))
        data.extend(bytearray(struct.pack('i',1)))
        frame.data = str(data)
        self._canPub.publish(frame)
        if self._started == True:
            print 'Science module has started'
            return [True, 'Science module has started']
        else:
            print 'Science module has stopped'
            return [True, 'Science module has stopped']

    #constructor
    def __init__(self):
        self._node = rospy.init_node('drill_control', anonymous=True)
        self._joySub = rospy.Subscriber('/joy', Joy, self.drillCallback)
        self._startService = rospy.Service('start_science', Trigger, self.start)
        self._canPub = rospy.Publisher('/CAN_transmitter', Frame, queue_size=20)
        self._elevator_dir = 0
        self._drill_duty = 0
        self._drill_id = 700 # CAN id
        self._start_id = 711
        self._started = False
        self._sent_zero = False
        print 'Started Drill Controller'

    def drillCallback(self, msg):
        if self._started == True:
            if msg.buttons[0] == 1:
                self._elevator_dir = 1
            elif msg.buttons[2] == 1:
                self._elevator_dir = -1
            else:
                self._elevator_dir = 0
            
            if msg.buttons[1] == 1:
                self._drill_duty = msg.axes[1] / 2.0
            else:
                self._drill_duty = 0

            # if msg.buttons[0] == 0 and msg.buttons[1] == 0 and msg.buttons[2] == 0:
            #     if self._sent_zero == False:
            #         self.sendCAN(False)
            #         self._sent_zero = True
            #         print 'Sending to Drill: Drill: 0 Elevator: 0'
            # else:
            print 'Sending to Drill: Drill: %f Elevator: %d' % (self._drill_duty, self._elevator_dir)
            self.sendCAN(True)
                # self._sent_zero = False

    def sendCAN(self, sendData):
        frame = Frame()
        frame.id = self._drill_id
        frame.is_rtr = False
        frame.is_error = False
        frame.is_extended = False
        frame.dlc = 8

        if sendData == False:
            data = bytearray(struct.pack('i', 0))
            data.extend(bytearray(struct.pack('i', 0)))
        else:
            data = bytearray(struct.pack('f', self._drill_duty))
            data.extend(bytearray(struct.pack('i', self._elevator_dir)))

        frame.data = str(data)
        self._canPub.publish(frame)

if __name__ == '__main__':
	controller = DrillController()
	rospy.spin()
