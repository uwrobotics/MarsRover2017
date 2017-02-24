#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "rover_joy/rover_joy_node.h"

ros::Publisher left_pub;
ros::Publisher right_pub;

const float MAX_X = 1.5;
const float MAX_Z = 1.0;

void twistCallback(const geometry_msgs::Twist::ConstPtr& vel)
{

	roboteq_msgs::Command left_command;
	roboteq_msgs::Command right_command;
	left_command.mode = MODE_VELOCITY;
	right_command.mode = MODE_VELOCITY;
	
	ROS_INFO("lin: %f ang: %f\r\n", vel->linear.x, vel->angular.z);
	
	int left = 0, right = 0;
	left = (vel->linear.x / MAX_X * 100.0) - (vel->angular.z / MAX_Z * 100);
	right = vel-> linear.x / MAX_X * 100.0 + (vel->angular.z / MAX_Z * 100);

	left_command.setpoint = left*10;
	right_command.setpoint = right*10;

	left_pub.publish(left_command);
	right_pub.publish(right_command);

	ROS_INFO("left: %d right: %d\r\b", left, right);
	
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rover_joy_node");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("cmd_vel", 100, twistCallback);
	left_pub = n.advertise<roboteq_msgs::Command&>("/left/cmd",1);
	right_pub = n.advertise<roboteq_msgs::Command&>("/right/cmd",1);
	
	ros::spin();
	return 0;
}
