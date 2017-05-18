//
//
//UW Robotics - Mars Rover Team
//Arm Control Executable
//
//University of Waterloo
//Waterloo, Ontario, Canada, Earth
//Code Author: Kieran Ratcliffe
//Last updated: May 10, 2017
//Last updated by: Kieran Ratcliffe


#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include <sstream>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"

const float DUTY_SAFETY_FACTOR = 1.0; //Safety factor to ensure microcontroller not outputting full duty cycle (set to 1.0 if not desired)
const int JOYSTICK_DEADZONE = 0;
const int NUM_DATA = 6; //Number of pieces of data to send to the arduino
const int CLAW_OPEN = 1.0; //Must either be 1 or -1, change so that pressing Right joystick, button 2 in claw mode OPENS the claw and button 3 CLOSES the claw

float dutyWriteClaw = 0;
float dutyWriteWristL = 0;
float dutyWriteWristR = 0;
float dutyWriteElbow = 0;
float dutyWriteShoulder = 0;
float dutyWriteTurntable = 0;

const int INDEX_LS_1 = 0; //sensor_msg.buttons index number for the 1 button/trigger of the left joystick
//INDEX_LS_1 acts as a Deadman's switch
const int INDEX_LS_2 = 1; //sensor_msg.buttons index number for the 2 button of the left joystick
const int INDEX_LS_3 = 2; //sensor_msg.buttons index number for the 3 button of the left joystick
const int INDEX_LS_UD = 1; //sensor_msg.axes index number for the up/down value from the left joystick
const int INDEX_LS_LR = 0; //sensor_msg.axes index number for the right/left value from the left joystick
const int INDEX_LS_DAMPER = 2; //sensor_msg.axes index number for the damper dial from the left joystick

const int INDEX_RS_1 = 0; //sensor_msg.buttons index number for the 1 button/trigger of the left joystick
//INDEX_RS_1 acts as the claw mode activator WHEN HELD DOWN
const int INDEX_RS_2 = 1; //sensor_msg.buttons index number for the 2 button of the left joystick
const int INDEX_RS_3 = 2; //sensor_msg.buttons index number for the 3 button of the left joystick
const int INDEX_RS_UD = 1; //sensor_msg.axes index number for the up/down value from the left joystick
const int INDEX_RS_LR = 0; //sensor_msg.axes index number for the right/left value from the left joystick
const int INDEX_RS_DAMPER = 2; //sensor_msg.axes index number for the damper dial from the left joystick

std_msgs::Int32MultiArray dutyWriteArray;

void clawCalculationsCoarse (const sensor_msgs::Joy& joy_msg);
void wristCalculationsCoarse (const sensor_msgs::Joy& joy_msg);
void shoulderCalculationsCoarse (const sensor_msgs::Joy& joy_msg);
void elbowCalculationsCoarse (const sensor_msgs::Joy& joy_msg);
void turntableCalculationsCoarse (const sensor_msgs::Joy& joy_msg);


void chatterCallback(const sensor_msgs::Joy& joy_msg){
	dutyWriteArray.data.clear();
	
	//if !deadman swtich
	//Do not run any arm controls
	
	//if fine control
	//insert function calls
	
	clawCalculationsCoarse(joy_msg);
	wristCalculationsCoarse(joy_msg);
	shoulderCalculationsCoarse(joy_msg);
	elbowCalculationsCoarse(joy_msg);
	turntableCalculationsCoarse(joy_msg);
	

	
	dutyWriteArray.data.push_back(dutyWriteWristL);
	dutyWriteArray.data.push_back(dutyWriteWristR);
	dutyWriteArray.data.push_back(dutyWriteElbow);
	dutyWriteArray.data.push_back(dutyWriteShoulder);
	dutyWriteArray.data.push_back(dutyWriteTurntable);
	dutyWriteArray.data.push_back(dutyWriteClaw);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "arm_controlv1");
	ros::NodeHandle n;
	dutyWriteArray.data.resize(NUM_DATA,0);
	
	  
	ros::Publisher chatter_pub = n.advertise<std_msgs::Int32MultiArray>("chatter", 1);
	ros::Subscriber sub = n.subscribe("/joy0/joy", 1, chatterCallback);
  
    ros::Rate loop_rate(10);
    int count = 0;
	while (ros::ok()) {
		
		
		ros::spinOnce();
		chatter_pub.publish(dutyWriteArray);
		loop_rate.sleep();
		++count;
	}

  return 0;
}

void clawCalculationsCoarse(const sensor_msgs::Joy& joy_msg){
	//Controls Algorithm for the Coarse Claw Movement (Default)
    //
    //
	//Control the speed of the motor by sending a PWM signal via the microcontroller
	//Output a value for a duty cycle on the range [-1,1], negative numbers correspond to a negative voltage
	//which gives the motor a negative velocity (the microcontroller handles this calculation)
	//
	//Default to output 0 (including initially writing a value of 0 in case of errors
	//Safety checks:
	////1. Deadman Switch (not implemented)
	////2. Kill Switch (not implemented)
	
	dutyWriteClaw = 0;
	if (joy_msg.buttons[INDEX_RS_1] == 1) {
		//Right trigger/button IS held down indicating claw mode
		if (joy_msg.buttons[INDEX_RS_2] == 1) { 
			//If both buttons pressed at the same time, overrides to openning for safety of claw
			dutyWriteClaw = CLAW_OPEN*DUTY_SAFETY_FACTOR; //Should OPEN the claw, change CLAW_OPEN if this isn't the case
		}
		else if (joy_msg.buttons[INDEX_RS_3] == 1) {
			dutyWriteClaw = (-1)*CLAW_OPEN*DUTY_SAFETY_FACTOR; //Should CLOSE the claw, change CLAW_OPEN if this isn't the case
		}
		else {
			dutyWriteClaw = 0;
		}
	}
	else { 
		//Right trigger/button 1 IS NOT held down, indicating it is in the default mode
		dutyWriteClaw = 0;
	}
}

void wristCalculationsCoarse(const sensor_msgs::Joy& joy_msg) {
	//Controls Algorithm for the Coarse Claw Roll and Pitch Movement (Default)
    //
    //
	//Control the speed of the motor by sending a PWM signal via the microcontroller
	//Output a value for a duty cycle on the range [-1,1], negative numbers correspond to a negative voltage
	//which gives the motor a negative velocity (the microcontroller handles this calculation)
	//
	//Default to output 0 (including initially writing a value of 0 in case of errors
	//Safety checks:
	////1. Deadman Switch (not implemented)
	////2. Kill Switch (not implemented)
	
	int ls_ud_detected = 0; //Integer representation of a boolean detecting whether up/down data has been
							//detected in left stick (i.e. input above 0.1 which is considered error)
	int ls_lr_detected = 0; //Integer representation of a boolean detecting whether left/right data has been
							//detected in left stick (i.e. input above 0.1 which is considered error)
	
	dutyWriteWristL = 0;
	dutyWriteWristR = 0;
	
	//Controls Algorithm for the Wrist Motors
    //The left analog stick is used to control both motors to achieve rotation in forward and lateral axes
	if (joy_msg.buttons[INDEX_RS_1] == 1) {
		//Right trigger/button IS held down indicating claw mode
		if ((abs(joy_msg.axes[INDEX_LS_UD]) >= JOYSTICK_DEADZONE) || (abs(joy_msg.axes[INDEX_LS_LR]) >= JOYSTICK_DEADZONE)) { 
			if ((abs(joy_msg.axes[INDEX_LS_UD]) >= JOYSTICK_DEADZONE)){//Detection criteria for up/down movement
				ls_ud_detected = 1;
			}
			if ((abs(joy_msg.axes[INDEX_LS_LR]) >= JOYSTICK_DEADZONE)){//Detection criteria for left/right movement
				ls_lr_detected = 1;
			}
			else {
				int ls_ud_detected = 0;
				int ls_lr_detected = 0; 
			}
			
			//Code that details the magnitude of the voltage sent to the motors
			//If up/down or left/right movement NOT detected, will not be considered in the calculation
			//If the analog stick is positioned straight up or down, there will only be rotation about the lateral axis, but
			//if the analog stick is positioned straight left or right, there will only be rotation about the forward axis
			dutyWriteWristL = (DUTY_SAFETY_FACTOR)*(joy_msg.axes[INDEX_LS_UD]*ls_ud_detected + joy_msg.axes[INDEX_LS_LR]*ls_lr_detected)/2;
			dutyWriteWristR = (DUTY_SAFETY_FACTOR)*(joy_msg.axes[INDEX_LS_UD]*ls_ud_detected - joy_msg.axes[INDEX_LS_LR]*ls_lr_detected)/2;
			
			
		}
		else {
			dutyWriteWristL = 0;
			dutyWriteWristR = 0;
		}
	}
	else { 
		//Right trigger/button 1 IS NOT held down, indicating it is in the default mode
		//The wrist is only capable of changing its pitch
		if ((abs(joy_msg.axes[INDEX_RS_LR]) >= JOYSTICK_DEADZONE)) { 
			dutyWriteWristL = DUTY_SAFETY_FACTOR*joy_msg.axes[INDEX_RS_LR];
			dutyWriteWristR = DUTY_SAFETY_FACTOR*joy_msg.axes[INDEX_RS_LR];
		}
		else {
			dutyWriteWristL = 0;
			dutyWriteWristR = 0;
		}
	}
}


void shoulderCalculationsCoarse(const sensor_msgs::Joy& joy_msg){
	//Controls Algorithm for the Coarse Shoulder Movement (Default)
    //
    //
	//Control the speed of the motor by sending a PWM signal via the microcontroller
	//Output a value for a duty cycle on the range [-1,1], negative numbers correspond to a negative voltage
	//which gives the motor a negative velocity (the microcontroller handles this calculation)
	//
	//Default to output 0 (including initially writing a value of 0 in case of errors
	//Safety checks:
	////1. Deadman Switch (not implemented)
	////2. Kill Switch (not implemented)
	
	dutyWriteShoulder = 0;
	if (joy_msg.buttons[INDEX_RS_1] == 1) {
		//Right trigger/button IS held down indicating claw mode
		if ((abs(joy_msg.buttons[INDEX_LS_UD]) >= JOYSTICK_DEADZONE)) { 
			dutyWriteShoulder = DUTY_SAFETY_FACTOR*joy_msg.axes[INDEX_LS_UD];
		}
		else {
			dutyWriteShoulder = 0;
		}
	}
	else { 
		//Right trigger/button 1 IS NOT held down, indicating it is in the default mode
		if ((abs(joy_msg.axes[INDEX_LS_UD]) >= JOYSTICK_DEADZONE)) { 
			dutyWriteShoulder = DUTY_SAFETY_FACTOR*joy_msg.axes[INDEX_LS_UD];
		}
		else {
			dutyWriteShoulder = 0;
		}
	}
}

void elbowCalculationsCoarse(const sensor_msgs::Joy& joy_msg){
	//Controls Algorithm for the Coarse Elbow Movement (Default)
    //
    //
	//Control the speed of the motor by sending a PWM signal via the microcontroller
	//Output a value for a duty cycle on the range [-1,1], negative numbers correspond to a negative voltage
	//which gives the motor a negative velocity (the microcontroller handles this calculation)
	//
	//Default to output 0 (including initially writing a value of 0 in case of errors
	//Safety checks:
	////1. Deadman Switch (not implemented)
	////2. Kill Switch (not implemented)
	
	dutyWriteElbow = 0;
	if (joy_msg.buttons[INDEX_RS_1] == 1) {
		//Right trigger/button IS held down indicating claw mode
		dutyWriteElbow = 0;
	}
	else { 
		//Right trigger/button 1 IS NOT held down, indicating it is in the default mode
		if ((abs(joy_msg.axes[INDEX_RS_UD]) >= JOYSTICK_DEADZONE)) { 
			dutyWriteElbow = DUTY_SAFETY_FACTOR*joy_msg.axes[INDEX_RS_UD];
		}
		else {
			dutyWriteElbow = 0;
		}
	}
}

void turntableCalculationsCoarse(const sensor_msgs::Joy& joy_msg) {
	//Controls Algorithm for the Coarse TurnTable Movement (Default)
    //
    //
	//Control the speed of the motor by sending a PWM signal via the microcontroller
	//Output a value for a duty cycle on the range [-1,1], negative numbers correspond to a negative voltage
	//which gives the motor a negative velocity (the microcontroller handles this calculation)
	//
	//Default to output 0 (including initially writing a value of 0 in case of errors
	//Safety checks:
	////1. Deadman Switch (not implemented)
	////2. Kill Switch (not implemented)
	
	dutyWriteTurntable = 1.0;
	if (joy_msg.buttons[INDEX_RS_1] == 1) {
		//Right trigger/button IS held down indicating claw mode
		if ((abs(joy_msg.axes[INDEX_LS_LR]) >= JOYSTICK_DEADZONE)) { 
			dutyWriteTurntable = DUTY_SAFETY_FACTOR*joy_msg.axes[INDEX_LS_LR];
		}
		else {
			dutyWriteTurntable = 1.0;
		}
	}
	else { 
		//Right trigger/button 1 IS NOT held down, indicating it is in the default mode
		if ((abs(joy_msg.axes[INDEX_LS_LR]) >= JOYSTICK_DEADZONE)) { 
			dutyWriteTurntable = DUTY_SAFETY_FACTOR*joy_msg.axes[INDEX_LS_LR];
		}
		else {
			dutyWriteTurntable = 1.0;
		}
	}
}











