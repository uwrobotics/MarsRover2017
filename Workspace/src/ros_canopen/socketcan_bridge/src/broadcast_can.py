import rospy
import roslib
from can_msgs.msg import Frame
from struct import *


frame = Frame()
frame.id = 5
frame.is_rtr = False
frame.is_extended = False
frame.is_error = False
frame.dlc = 4
data = bytearray(pack('f', 1.2345))
frame.data = str(data)
print (frame)
# pub = rospy.Publisher("/can_test", Frame, queue_size=10)
# rospy.init_node('talker', anonymous=True)
# rate = rospy.Rate(10)
# while not rospy.is_shutdown():
# 	pub.publish(frame)
# 	rate.sleep()