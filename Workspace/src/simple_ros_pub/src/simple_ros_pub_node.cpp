#include "ros/ros.h"
#include "std_msgs/Int32.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "simple_ros_pub");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<std_msgs::Int32>("simple_msg", 1000);
	ros::Rate loop_rate(1);
	int i = 0;
	while(ros::ok())
	{
		i++;
		std_msgs::Int32 msg;
		msg.data = i;
		ROS_INFO("%d", msg.data);
		chatter_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
