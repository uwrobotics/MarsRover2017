#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Float32.h"
#include "can_msgs/Frame.h"

#define SWITCHES_MASK_BASE 0xF
#define SWITCHES_MASK1 0x1
#define SWITCHES_MASK2 0x2
#define SWITCHES_MASK3 0x4
#define SWITCHES_MASK4 0x8


std_msgs::UInt8 flagMsg;
float shoulderPosition = 0;
float limitValue = 10;

void chatterCallback(const std_msgs::Float32& floatMsg) {
	uint8_t temp8 = 0;
	temp8 &= SWITCHES_MASK_BASE;
	
	float temp = floatMsg.data;
	shoulderPosition += temp;
	
	if (shoulderPosition > limitValue) {
		temp8 = 4;
	}
	else if (shoulderPosition < -limitValue) {
		temp8 = 8;
	}
	else {
		temp8 = 0;
	}
	
	flagMsg.data = temp8;
	
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "testPublisher");
    ros::NodeHandle n;
    //dutyWriteArray.data.resize(NUM_DATA,0);
    
    //ros::Publisher chatter_pub = n.advertise<can_msgs::Frame>("/CAN_transmitter", 30);
    ros::Subscriber sub = n.subscribe("/testTopic", 1, chatterCallback);
    ros::Publisher switchPub = n.advertise<std_msgs::UInt8>("switchesShoulderFlags", 1);
  
  
    ros::Rate loop_rate(10);
    int count = 0;
    while (ros::ok()) {
        ros::spinOnce();
        
        switchPub.publish(flagMsg);
		
        loop_rate.sleep();
        ++count;
    }

  return 0;
}