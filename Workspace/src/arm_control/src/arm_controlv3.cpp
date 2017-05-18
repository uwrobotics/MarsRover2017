//
//
//UW Robotics - Mars Rover Team
//Arm Control Executable
//
//University of Waterloo
//Waterloo, Ontario, Canada, Earth
//Code Author: Kieran Ratcliffe
//Last updated: May 16, 2017
//Last updated by: Kieran Ratcliffe


#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Int32MultiArray.h"
#include "can_msgs/Frame.h"


const float DUTY_SAFETY_FACTOR =1.0 //Safety factor to ensure microcontroller not outputting full duty cycle (set to 1.0 if not desired)
const int JOYSTICK_DEADZONE = 0;
const int NUM_DATA = 6; //Number of pieces of data to send to the arduino
const int CLAW_OPEN = 1.0; //Must either be 1 or -1, change so that pressing Right joystick, button 2 in claw mode OPENS the claw and button 3 CLOSES the claw
const int NUM_AXES_DATA = 3;
const int NUM_BUTTONS_DATA = 10;
const int NUM_BYTES_IN_FLOAT = 4;
const int ID_WRIST_FRAME = 68;
const int ID_LIMBS_FRAME = 69;
const int ID_OTHERS_FRAME = 70;


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
can_msgs::Frame CANFrame;
can_msgs::Frame wristsCANFrame; //First 4 bytes = left wrist, second 4 bytes = right wrist
can_msgs::Frame limbsCANFrame; //First 4 bytes = elbow, second 4 bytes = shoulder
can_msgs::Frame othersCANFrame; //First 4 bytes = turntable, second 4 bytes = claw
float axesRight[NUM_AXES_DATA] = {};
float axesLeft[NUM_AXES_DATA] = {};
float buttonsRight[NUM_BUTTONS_DATA] = {};
float buttonsLeft[NUM_BUTTONS_DATA] = {};
float outputPWM[NUM_DATA] = {};

void clawCalculationsCoarse ();
void wristCalculationsCoarse ();
void shoulderCalculationsCoarse ();
void elbowCalculationsCoarse ();
void turntableCalculationsCoarse ();
void chatterHelper();
void getMessageAxesData(const sensor_msgs::Joy& joy_msg, float arr[], int size);
void getMessageButtonsData(const sensor_msgs::Joy& joy_msg, float arr[], int size);
void float2Bytes(float val,uint8_t* bytes_array);
can_msgs::Frame populateFrame(can_msgs::Frame frameMsg, uint8_t bytesArray1[NUM_BYTES_IN_FLOAT], uint8_t bytesArray2[NUM_BYTES_IN_FLOAT]);
void frameFormerHelper();

void chatterCallback0(const sensor_msgs::Joy& joy_msgR){
	//dutyWriteArray.data.clear();
	for (int k = 0; k < NUM_DATA; k++) {
		outputPWM[k] = 0;
	}
	getMessageAxesData(joy_msgR,axesRight,NUM_AXES_DATA);
	getMessageButtonsData(joy_msgR,buttonsRight,NUM_BUTTONS_DATA);
	
	//if !deadman swtich
	//Do not run any arm controls
	if (buttonsLeft[INDEX_LS_1] == 1) {
			
		//if fine control
		//insert function calls
		chatterHelper();
		
		outputPWM[0] = dutyWriteWristL;
		outputPWM[1] = dutyWriteWristR;
		outputPWM[2] = dutyWriteElbow;
		outputPWM[3] = dutyWriteShoulder;
		outputPWM[4] = dutyWriteTurntable;
		outputPWM[5] = dutyWriteClaw;
	}

}

void chatterCallback1(const sensor_msgs::Joy& joy_msgL){
	//dutyWriteArray.data.clear();
	for (int k = 0; k < NUM_DATA; k++) {
		outputPWM[k] = 0;
	}
	getMessageAxesData(joy_msgL,axesLeft,NUM_AXES_DATA);
	getMessageButtonsData(joy_msgL,buttonsLeft,NUM_BUTTONS_DATA);
	
	//if !deadman swtich
	//Do not run any arm controls
	if (buttonsLeft[INDEX_LS_1] == 1) {
			
		//if fine control
		//insert function calls
		chatterHelper();
		
		outputPWM[0] = dutyWriteWristL;
		outputPWM[1] = dutyWriteWristR;
		outputPWM[2] = dutyWriteElbow;
		outputPWM[3] = dutyWriteShoulder;
		outputPWM[4] = dutyWriteTurntable;
		outputPWM[5] = dutyWriteClaw;
	}
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "arm_controlv3");
	ros::NodeHandle n;
	//dutyWriteArray.data.resize(NUM_DATA,0);
	
	
	//ros::Publisher chatter_pub = n.advertise<std_msgs::Int32MultiArray>("chatter", 1);
	ros::Publisher chatter_pub = n.advertise<can_msgs::Frame>("chatter", 1);
	ros::Publisher chatter_pub2 = n.advertise<can_msgs::Frame>("chatter2", 1);
	ros::Publisher chatter_pub3 = n.advertise<can_msgs::Frame>("chatter3", 1);

	ros::Subscriber sub0 = n.subscribe("/joy0/joy", 1, chatterCallback0);
	ros::Subscriber sub1 = n.subscribe("/joy1/joy", 1, chatterCallback1);
  
    ros::Rate loop_rate(10);
    int count = 0;
	while (ros::ok()) {
		ros::spinOnce();
		
		frameFormerHelper();
		chatter_pub.publish(wristsCANFrame);
		chatter_pub.publish(limbsCANFrame);
		chatter_pub.publish(othersCANFrame);
		
		loop_rate.sleep();
		++count;
	}

  return 0;
}

void clawCalculationsCoarse(){
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
	if (buttonsRight[INDEX_RS_1] == 1) {
		//Right trigger/button IS held down indicating claw mode
		if (buttonsRight[INDEX_RS_2] == 1) { 
			//If both buttons pressed at the same time, overrides to openning for safety of claw
			dutyWriteClaw = CLAW_OPEN*DUTY_SAFETY_FACTOR; //Should OPEN the claw, change CLAW_OPEN if this isn't the case
		}
		else if (buttonsRight[INDEX_RS_3] == 1) {
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

void wristCalculationsCoarse() {
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
	
	dutyWriteWristL = 0;
	dutyWriteWristR = 0;
	
	//Controls Algorithm for the Wrist Motors
    //The left analog stick is used to control both motors to achieve rotation in forward and lateral axes
	if (buttonsRight[INDEX_RS_1] == 1) {
		//Right trigger/button IS held down indicating claw mode
		if ((abs(axesRight[INDEX_RS_UD]) >= JOYSTICK_DEADZONE) || (abs(axesRight[INDEX_RS_LR]) >= JOYSTICK_DEADZONE)) { 
			if ((abs(axesRight[INDEX_RS_UD]) >= JOYSTICK_DEADZONE)){//Detection criteria for up/down movement
				dutyWriteWristL = dutyWriteWristL + axesRight[INDEX_RS_UD];
				dutyWriteWristR = dutyWriteWristR + axesRight[INDEX_RS_UD];
			}
			if ((abs(axesRight[INDEX_RS_LR]) >= JOYSTICK_DEADZONE)){//Detection criteria for left/right movement
				dutyWriteWristL = dutyWriteWristL + axesRight[INDEX_RS_LR];
				dutyWriteWristR = dutyWriteWristR - axesRight[INDEX_RS_LR];
			}
			
			//Code that details the magnitude of the voltage sent to the motors
			//If up/down or left/right movement NOT detected, will not be considered in the calculation
			//If the analog stick is positioned straight up or down, there will only be rotation about the lateral axis, but
			//if the analog stick is positioned straight left or right, there will only be rotation about the forward axis
			dutyWriteWristL = (DUTY_SAFETY_FACTOR)*(dutyWriteWristL)/2;
			dutyWriteWristR = (DUTY_SAFETY_FACTOR)*(dutyWriteWristR)/2;
		}
		else {
			dutyWriteWristL = 0;
			dutyWriteWristR = 0;
		}
	}
	else { 
		//Right trigger/button 1 IS NOT held down, indicating it is in the default mode
		//The wrist is only capable of changing its pitch
		if ((abs(axesRight[INDEX_RS_LR]) >= JOYSTICK_DEADZONE)) { 
			dutyWriteWristL = DUTY_SAFETY_FACTOR*axesRight[INDEX_RS_LR];
			dutyWriteWristR = DUTY_SAFETY_FACTOR*axesRight[INDEX_RS_LR];
		}
		else {
			dutyWriteWristL = 0;
			dutyWriteWristR = 0;
		}
	}
	
	//Code use to error check for illegal values of dutyWriteWristL and dutyWriteWristR
	if (abs(dutyWriteWristL) > 1) {
		dutyWriteWristL = 1;
	}
	if (abs(dutyWriteWristR) > 1) {
		dutyWriteWristR = 1;
	}
}


void shoulderCalculationsCoarse(){
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
	if (buttonsRight[INDEX_RS_1] == 1) {
		//Right trigger/button IS held down indicating claw mode
		if ((abs(axesLeft[INDEX_LS_UD]) >= JOYSTICK_DEADZONE)) { 
			dutyWriteShoulder = DUTY_SAFETY_FACTOR*axesLeft[INDEX_LS_UD];
		}
		else {
			dutyWriteShoulder = 0;
		}
	}
	else { 
		//Right trigger/button 1 IS NOT held down, indicating it is in the default mode
		if ((abs(axesLeft[INDEX_LS_UD]) >= JOYSTICK_DEADZONE)) { 
			dutyWriteShoulder = DUTY_SAFETY_FACTOR*axesLeft[INDEX_LS_UD];
		}
		else {
			dutyWriteShoulder = 0;
		}
	}
}

void elbowCalculationsCoarse(){
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
	if (buttonsRight[INDEX_RS_1] == 1) {
		//Right trigger/button IS held down indicating claw mode
		dutyWriteElbow = 0;
	}
	else { 
		//Right trigger/button 1 IS NOT held down, indicating it is in the default mode
		if ((abs(axesRight[INDEX_RS_UD]) >= JOYSTICK_DEADZONE)) { 
			dutyWriteElbow = DUTY_SAFETY_FACTOR*axesRight[INDEX_RS_UD];
		}
		else {
			dutyWriteElbow = 0;
		}
	}
}

void turntableCalculationsCoarse() {
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
	if (buttonsRight[INDEX_RS_1] == 1) {
		//Right trigger/button IS held down indicating claw mode
		if ((abs(axesLeft[INDEX_LS_LR]) >= JOYSTICK_DEADZONE)) { 
			dutyWriteTurntable = DUTY_SAFETY_FACTOR*axesLeft[INDEX_LS_LR];
		}
		else {
			dutyWriteTurntable = 1.0;
		}
	}
	else { 
		//Right trigger/button 1 IS NOT held down, indicating it is in the default mode
		if ((abs(axesLeft[INDEX_LS_LR]) >= JOYSTICK_DEADZONE)) { 
			dutyWriteTurntable = DUTY_SAFETY_FACTOR*axesLeft[INDEX_LS_LR];
		}
		else {
			dutyWriteTurntable = 1.0;
		}
	}
}

void chatterHelper(){
	clawCalculationsCoarse();
	wristCalculationsCoarse();
	shoulderCalculationsCoarse();
	elbowCalculationsCoarse();
	turntableCalculationsCoarse();
}

//Set the values of the global array to the values in the joy_msg
void getMessageAxesData(const sensor_msgs::Joy& joy_msg, float arr[], int size) {
	for (int i=0; i <= size; i++) {
		arr[i] = joy_msg.axes[i];
	}
}

//Set the values of the global array to the values in the joy_msg
void getMessageButtonsData(const sensor_msgs::Joy& joy_msg, float arr[], int size) {
	for (int i=0; i <= size; i++) {
		arr[i] = joy_msg.buttons[i];
	}
}

void float2Bytes(float val,uint8_t* bytes_array){
  // Create union of shared memory space
  union {
    float float_variable;
    uint8_t temp_array[4];
  } u;
  // Overite bytes of union with float variable
  u.float_variable = val;
  // Assign bytes to input array
  memcpy(bytes_array, u.temp_array, 4);
}

can_msgs::Frame populateFrame(can_msgs::Frame frameMsg, uint8_t bytesArray1[NUM_BYTES_IN_FLOAT], uint8_t bytesArray2[NUM_BYTES_IN_FLOAT]){
	frameMsg.data[0] = (bytesArray1[0]);
	frameMsg.data[1] = (bytesArray1[1]);
	frameMsg.data[2] = (bytesArray1[2]);
	frameMsg.data[3] = (bytesArray1[3]);
	frameMsg.data[4] = (bytesArray2[0]);
	frameMsg.data[5] = (bytesArray2[1]);
	frameMsg.data[6] = (bytesArray2[2]);
	frameMsg.data[7] = (bytesArray2[3]);
	return frameMsg;
}

void frameFormerHelper(){
			
		struct motorBytesStruct{
			uint8_t firstMotorBytes[NUM_BYTES_IN_FLOAT], secondMotorBytes[NUM_BYTES_IN_FLOAT];
		} wristMotorBytes, limbMotorBytes, otherMotorBytes;
		
		float2Bytes(outputPWM[0],&wristMotorBytes.firstMotorBytes[0]);
		float2Bytes(outputPWM[1],&wristMotorBytes.secondMotorBytes[0]);
		float2Bytes(outputPWM[2],&limbMotorBytes.firstMotorBytes[0]);
		float2Bytes(outputPWM[3],&limbMotorBytes.secondMotorBytes[0]);
		float2Bytes(outputPWM[4],&otherMotorBytes.firstMotorBytes[0]);
		float2Bytes(outputPWM[5],&otherMotorBytes.secondMotorBytes[0]);
		wristsCANFrame = populateFrame(wristsCANFrame, wristMotorBytes.firstMotorBytes, wristMotorBytes.secondMotorBytes);
		limbsCANFrame = populateFrame(limbsCANFrame, limbMotorBytes.firstMotorBytes, limbMotorBytes.secondMotorBytes);
		othersCANFrame = populateFrame(othersCANFrame, otherMotorBytes.firstMotorBytes, otherMotorBytes.secondMotorBytes);

		wristsCANFrame.id = ID_WRIST_FRAME;
		limbsCANFrame.id = ID_LIMBS_FRAME;
		othersCANFrame.id = ID_OTHERS_FRAME;
}