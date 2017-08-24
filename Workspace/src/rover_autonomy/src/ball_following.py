#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist

import math

# the velocity and following distance of the rover, need to be changed to the real value
velocity = 1.0
following_distance = 1.5

pixel_size = 3.75 * 10**(-6) # in m
focal_length = 8 * 10**(-3) # in m
image_width = 1288
image_height = 964

max_angle = math.atan(image_width / 2.0 * pixel_size / focal_length)


def callback(array):
    if array.data[0] == 0:
        return

    msg = Twist()

    msg.linear.x = velocity if array.data[3] > following_distance else 0 #z
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = max_angle + array.data[4] #bearing
    # msg.angular.z = - (array.data[4] + max_angle / 2)
    
    follow_pub.publish(msg)


def main():
    global follow_pub
    ball_sub = rospy.Subscriber("/tennisball", Float64MultiArray, callback)
    rospy.init_node('ball_following', anonymous=True)
    follow_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

    rospy.spin()


if __name__ == '__main__':
    main()
