#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "simple_ros_pub");
	ros::NodeHandle n;
	ros::Publisher chatter_pub = n.advertise<std_msgs::Float32MultiArray>("battery_data", 1000);
	ros::Publisher pub2 = n.advertise<std_msgs::Float32MultiArray>("temperature_data", 1000);
	ros::Rate loop_rate(2);
	float p = 100;
	float c = 0;
	float v = 12;
	float outside = 25;
	float cpu = 30;
	float robo = 33;
	int cnt = 0;
	while(ros::ok())
	{
		cnt++;
		p -= 0.5;
		c += 0.5;
		v -= 0.011;
		
		if(cnt >= 4){
		outside -= 0.2;
		cpu += 0.4;
		robo += 0.3;
		cnt=0;
		}
		std_msgs::Float32MultiArray msg;
		std_msgs::Float32MultiArray temp_msg;
		msg.data.push_back(p);
		msg.data.push_back(c);
		msg.data.push_back(v);
		temp_msg.data.push_back(outside);
		temp_msg.data.push_back(cpu);
		temp_msg.data.push_back(robo);
		chatter_pub.publish(msg);
		pub2.publish(temp_msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
