#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "rover_joy/rover_joy_node.h"

ros::Publisher left_pub;
ros::Publisher right_pub;

const float MAX_X = 2.0;   // m/s
const float MAX_Z = 2.0; // rad/s
const float SCALE = 1000;  // max value for roboteq
const float WHEEL_DISTANCE = 0.865;
const float RADIUS = 0.105;
const float MAX_WHEEL_SPEED = (MAX_X + WHEEL_DISTANCE/2 * MAX_Z)/RADIUS;


void twistCallback(const geometry_msgs::Twist::ConstPtr& vel)
{

	roboteq_msgs::Command left_command;
	roboteq_msgs::Command right_command;
	left_command.mode = MODE_VELOCITY;
	right_command.mode = MODE_VELOCITY;
	
	ROS_INFO("lin: %f ang: %f", vel->linear.x, vel->angular.z);
	
	float left = 0, right = 0;
	left = ((vel->linear.x / MAX_X) - (vel->angular.z / MAX_Z))*SCALE;
	right = ((vel-> linear.x / MAX_X) + (vel->angular.z / MAX_Z))*SCALE;

	left = left > SCALE ? SCALE : left;
	right = right > SCALE ? SCALE : right;

	if(vel->linear.x < 0) {
		float temp = left;
		left = right;
		right = temp;
	}
	//left = ((vel->linear.x - WHEEL_DISTANCE/2 * vel->angular.z)/RADIUS) / MAX_WHEEL_SPEED * SCALE;
	//right = ((vel->linear.x + WHEEL_DISTANCE/2 * vel->angular.z)/RADIUS) / MAX_WHEEL_SPEED * SCALE;

	left_command.setpoint = left;
	right_command.setpoint = right;

	left_pub.publish(left_command);
	right_pub.publish(right_command);

	ROS_INFO("left: %f right: %f\r\n\r\n", left, right);
	
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
