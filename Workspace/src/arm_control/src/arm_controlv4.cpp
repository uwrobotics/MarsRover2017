//
//
//UW Robotics - Mars Rover Team
//Arm Control Executable
//
//University of Waterloo
//Waterloo, Ontario, Canada, Earth
//Code Author: Kieran Ratcliffe
//Last updated: May 24, 2017
//Last updated by: Kieran Ratcliffe


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

std_msgs::Float32 testMsg;


//***OMG SRS DO NOT MAKE THIS VALUE LARGER THAN 0.5!!***
const float DUTY_SAFETY_FACTOR = 0.5; //Safety factor to ensure motorcontrollers do not break the motors
//***OMG SRS DO NOT MAKE THIS VALUE LARGER THAN 0.5!!***
//Values larger than 0.5 will output a value larger than 12V to the motors which will break them!!

const int JOYSTICK_DEADZONE = 0;
const int NUM_DATA = 6; //Total number of pieces of data to send to the arm boards
const int CLAW_OPEN = 1.0; //Must either be 1 or -1, change so that pressing Right joystick, button 2 in claw mode OPENS the claw and button 3 CLOSES the claw
const int NUM_AXES_DATA = 3;
const int NUM_BUTTONS_DATA = 10;
const int NUM_BYTES_IN_FLOAT = 4;
const int ID_WRIST_FRAME = 68;
const int ID_FOREARM_FRAME = 69;
const int ID_SHOULDER_FRAME = 70;


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
can_msgs::Frame forearmCANFrame; //First 4 bytes = elbow, second 4 bytes = claw
can_msgs::Frame shoulderCANFrame; //First 4 bytes = shoulder, second 4 bytes = turntable
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

bool isLimitTTCWTrip = false; //Is the clockwise limit switch for the turntable tripped
bool isLimitTTCCWTrip = false; //Is the counter-clockwise limit switch for the turntable tripped
bool isLimitShForwardTrip = false; //Is the forward limit switch for the shoulder tripped
bool isLimitShBackwardTrip = false; //Is the backward limit switch for the shoulder tripped
bool isLimitElbForwardTrip = false; //Is the forward limit switch for the elbow tripped
bool isLimitElbBackwardTrip = false; //Is the backward limit switch for the elbow tripped

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
        outputPWM[3] = dutyWriteClaw;
        outputPWM[4] = dutyWriteShoulder;
        outputPWM[5] = dutyWriteTurntable;
		
		for (int i = 0; i < NUM_DATA; i++) {
			outputPWM[i] = DUTY_SAFETY_FACTOR*outputPWM[i];
			if (abs(outputPWM[i]) > DUTY_SAFETY_FACTOR) {
				outputPWM[i] = DUTY_SAFETY_FACTOR*outputPWM[i]/abs(outputPWM[i]);
			}
		}
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
        outputPWM[3] = dutyWriteClaw;
        outputPWM[4] = dutyWriteShoulder;
        outputPWM[5] = dutyWriteTurntable;
		
		for (int i = 0; i < NUM_DATA; i++) {
			outputPWM[i] = DUTY_SAFETY_FACTOR*outputPWM[i];
			if (abs(outputPWM[i]) > DUTY_SAFETY_FACTOR) {
				outputPWM[i] = DUTY_SAFETY_FACTOR*outputPWM[i]/abs(outputPWM[i]);
			}
		}
    }
}

void switchCallbackWrist(const std_msgs::UInt8& wristSwitches) {

}

void switchCallbackForearm(const std_msgs::UInt8& forearmSwitches) {
	uint8_t temp = forearmSwitches.data;
	temp &= SWITCHES_MASK_BASE;
	
	//Bits in the byte message from shoulder board should be organized as such:
	//1 = elbow forward switch, 2 = elbow backward switch
	//All other bits unassigned
	if (temp & SWITCHES_MASK1) {
		isLimitElbForwardTrip = true;
	}
	else {
		isLimitElbForwardTrip = false;		
	}
	
	if (temp & SWITCHES_MASK2 >> 1)  {
		isLimitElbBackwardTrip = true;
	}
	else {
		isLimitElbBackwardTrip = false;		
	}
}

void switchCallbackShoulder(const std_msgs::UInt8& shoulderSwitches) {
	uint8_t temp = shoulderSwitches.data;
	temp &= SWITCHES_MASK_BASE;
	
	
	//Bits in the byte message from shoulder board should be organized as such:
	//1 = turntable clockwise switch, 2 = turntable counter-clockwise switch
	//3 = shoulder forward switch, 4 = shoulder backward switch, 5-8 n/a
	if (temp & SWITCHES_MASK1) {
		isLimitTTCWTrip = true;
	}
	else {
		isLimitTTCWTrip = false;		
	}
	
	if (temp & SWITCHES_MASK2 >> 1)  {
		isLimitTTCCWTrip = true;
	}
	else {
		isLimitTTCCWTrip = false;		
	}
	if (temp & SWITCHES_MASK3) {
		isLimitShForwardTrip = true;
	}
	else {
		isLimitShForwardTrip = false;		
	}
	if (temp & SWITCHES_MASK4)  {
		isLimitShBackwardTrip = true;
	}
	else {
		isLimitShBackwardTrip = false;		
	}
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "arm_controlv4");
    ros::NodeHandle n;
    //dutyWriteArray.data.resize(NUM_DATA,0);
    
    ros::Publisher chatter_pub = n.advertise<can_msgs::Frame>("/CAN_transmitter", 30);
    ros::Publisher testPub = n.advertise<std_msgs::Float32>("/testTopic", 1);

    ros::Subscriber sub0 = n.subscribe("/joy0/joy", 1, chatterCallback0);
    ros::Subscriber sub1 = n.subscribe("/joy1/joy", 1, chatterCallback1);
    ros::Subscriber switchSub1 = n.subscribe("switchesWristFlags", 1, switchCallbackWrist);
    ros::Subscriber switchSub2 = n.subscribe("switchesForearmFlags", 1, switchCallbackForearm);
    ros::Subscriber switchSub3 = n.subscribe("switchesShoulderFlags", 1, switchCallbackShoulder);
  
  
    ros::Rate loop_rate(10);
    int count = 0;
    while (ros::ok()) {
        ros::spinOnce();
		
		testMsg.data = outputPWM[4];
		
        frameFormerHelper();
        chatter_pub.publish(wristsCANFrame);
        //chatter_pub.publish(forearmCANFrame);
        //chatter_pub.publish(shoulderCANFrame);
		testPub.publish(testMsg);
        
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
            dutyWriteClaw = CLAW_OPEN; //Should OPEN the claw, change CLAW_OPEN if this isn't the case
        }
        else if (buttonsRight[INDEX_RS_3] == 1) {
            dutyWriteClaw = (-1)*CLAW_OPEN; //Should CLOSE the claw, change CLAW_OPEN if this isn't the case
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
            dutyWriteWristL = (dutyWriteWristL)/2;
            dutyWriteWristR = (dutyWriteWristR)/2;
        }
        else {
            dutyWriteWristL = 0;
            dutyWriteWristR = 0;
        }
    }
    else { 
        //Right trigger/button 1 IS NOT held down, indicating it is in the default mode
        //The wrist is only capable of changing its pitch
		if (buttonsRight[INDEX_RS_2] == 1) { 
            //If both buttons pressed at the same time, overrides to openning for safety of claw
            dutyWriteWristL = buttonsRight[INDEX_RS_2];
            dutyWriteWristR = buttonsRight[INDEX_RS_2];
        }
        else if (buttonsRight[INDEX_RS_3] == 1) {
			dutyWriteWristL = -buttonsRight[INDEX_RS_3];
            dutyWriteWristR = -buttonsRight[INDEX_RS_3];        
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
            dutyWriteShoulder = axesLeft[INDEX_LS_UD];
        }
        else {
            dutyWriteShoulder = 0;
        }
    }
    else { 
        //Right trigger/button 1 IS NOT held down, indicating it is in the default mode
        if ((abs(axesLeft[INDEX_LS_UD]) >= JOYSTICK_DEADZONE)) { 
            dutyWriteShoulder = axesLeft[INDEX_LS_UD];
        }
        else {
            dutyWriteShoulder = 0;
        }
    }

	if (isLimitShForwardTrip && (dutyWriteShoulder > 0)) {
		dutyWriteShoulder = 0;
	}
	if (isLimitShBackwardTrip && (dutyWriteShoulder < 0)) {
		dutyWriteShoulder = 0;
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
            dutyWriteElbow = axesRight[INDEX_RS_UD];
        }
        else {
            dutyWriteElbow = 0;
        }
    }
	
	if (isLimitElbForwardTrip && (dutyWriteElbow > 0)) {
		dutyWriteElbow = 0;
	}
	if (isLimitElbBackwardTrip && (dutyWriteElbow < 0)) {
		dutyWriteElbow = 0;
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
    
    dutyWriteTurntable = 0;
    if (buttonsRight[INDEX_RS_1] == 1) {
        //Right trigger/button IS held down indicating claw mode
        if ((abs(axesLeft[INDEX_LS_LR]) >= JOYSTICK_DEADZONE)) { 
            dutyWriteTurntable = axesLeft[INDEX_LS_LR];
        }
        else {
            dutyWriteTurntable = 0;
        }
    }
    else { 
        //Right trigger/button 1 IS NOT held down, indicating it is in the default mode
        if ((abs(axesLeft[INDEX_LS_LR]) >= JOYSTICK_DEADZONE)) { 
            dutyWriteTurntable = axesLeft[INDEX_LS_LR];
        }
        else {
            dutyWriteTurntable = 0;
        }
    }

	if (isLimitTTCWTrip && (dutyWriteTurntable > 0)) {
		dutyWriteTurntable = 0;
	}
	if (isLimitTTCWTrip && (dutyWriteTurntable < 0)) {
		dutyWriteTurntable = 0;
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

void populateFrame(can_msgs::Frame *frameMsg, float val1, float val2) {
    memcpy(&frameMsg->data[0], (unsigned char*) (&val1), 4);
    memcpy(&frameMsg->data[4], (unsigned char*) (&val2), 4);
}


void frameFormerHelper() {
        populateFrame(&wristsCANFrame, outputPWM[0], outputPWM[1]);
        wristsCANFrame.id = ID_WRIST_FRAME;
        wristsCANFrame.dlc = 8;
        wristsCANFrame.is_error = 0;
        wristsCANFrame.is_rtr = 0;
        wristsCANFrame.is_extended = 0;

        populateFrame(&forearmCANFrame, outputPWM[2], outputPWM[3]);
        forearmCANFrame.id = ID_FOREARM_FRAME;
        forearmCANFrame.dlc = 8;
        forearmCANFrame.is_error = 0;
        forearmCANFrame.is_rtr = 0;
        forearmCANFrame.is_extended = 0;

        populateFrame(&shoulderCANFrame, outputPWM[4], outputPWM[5]);
        shoulderCANFrame.id = ID_SHOULDER_FRAME;
        shoulderCANFrame.dlc = 8;
        shoulderCANFrame.is_error = 0;
        shoulderCANFrame.is_rtr = 0;
        shoulderCANFrame.is_extended = 0;
}