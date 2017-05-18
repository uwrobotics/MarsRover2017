#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"
#include <sstream>

#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"


const int DUTY_FULL_ACTUAL = 220;//In reality, 255 is the full duty cycle analogwrite value but reduced for safety purposes
const float DUTY_FULL_SCALE = 0.7; //Equal to 1/sqrt(2) needed for safety of wrist motors from calculations
const int DUTY_FULL = floor(DUTY_FULL_ACTUAL*DUTY_FULL_SCALE);
const int JOYSTICK_ERROR_THRESHOLD = 0.3;
const int SERVOPULSE_RANGE = 150; //In microseconds, the difference between full stop and full forward/reverse
const int SERVOPULSE_FULLSTOP = 1450; //In microseconds, the period of pulses for which the motor is fully stopped
const int SERVOPULSE_FULLSTOP_TT = 1450;
const int NUM_DATA = 10; //Number of pieces of data to send to the arduino

int analog_write_claw = 0;
int dir_claw_A = 0;
int analog_write_wrist1 = 0;
int dir_wrist1_high = 0;
int analog_write_wrist2 = 0;
int dir_wrist2_high = 0;
int analog_write_elbow = 0;
int dir_elbow_high = 0;
int servo_write_shoulder = 0;
int servo_write_turntable = 0;

const int INDEX_A = 0; //sensor_msg.buttons index number for the left bumper
const int INDEX_B = 1; //sensor_msg.buttons index number for the right bumper
const int INDEX_LB = 4; //sensor_msg.buttons index number for the left bumper
const int INDEX_RB = 5; //sensor_msg.buttons index number for the right bumper
const int INDEX_LT = 2; //sensor_msg.axes index number for the left trigger
const int INDEX_RT = 5; //sensor_msg.axes index number for the right trigger
const int INDEX_LS_UD = 1; //sensor_msg.axes index number for the up/down value from the left joystick
const int INDEX_LS_LR = 0; //sensor_msg.axes index number for the right/left value from the left joystick
const int INDEX_RS_UD = 4; //sensor_msg.axes index number for the up/down value from the right joystick
const int INDEX_RS_LR = 3; //sensor_msg.axes index number for the right/left value from the right joystick

std_msgs::Int32MultiArray analog_write_array;

void wrist_calcuations(const sensor_msgs::Joy& xbox_msg);
void shoulderelbow_calcuations(const sensor_msgs::Joy& xbox_msg);
void claw_calculations (const sensor_msgs::Joy& xbox_msg);
void turntable_calculations (const sensor_msgs::Joy& xbox_msg);

void chatterCallback(const sensor_msgs::Joy& xbox_msg){
	analog_write_array.data.clear();
		
	wrist_calcuations(xbox_msg);
	shoulderelbow_calcuations(xbox_msg);
	claw_calculations(xbox_msg);
	turntable_calculations(xbox_msg);
	

	analog_write_array.data.push_back(analog_write_wrist1);
	analog_write_array.data.push_back(dir_wrist1_high);
	analog_write_array.data.push_back(analog_write_wrist2);
	analog_write_array.data.push_back(dir_wrist2_high);
	analog_write_array.data.push_back(analog_write_elbow);
	analog_write_array.data.push_back(dir_elbow_high);
	analog_write_array.data.push_back(analog_write_claw);
	analog_write_array.data.push_back(dir_claw_A);
	analog_write_array.data.push_back(servo_write_shoulder);
	analog_write_array.data.push_back(servo_write_turntable);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "listener_arm");
	ros::NodeHandle n;
	analog_write_array.data.resize(NUM_DATA,0);
	
	  
	ros::Publisher chatter_pub = n.advertise<std_msgs::Int32MultiArray>("chatter", 1);
	ros::Subscriber sub = n.subscribe("joy", 1, chatterCallback);
  
    ros::Rate loop_rate(10);
    int count = 0;
	while (ros::ok()) {
		
		
		ros::spinOnce();
		chatter_pub.publish(analog_write_array);
		loop_rate.sleep();
		++count;
	}

  return 0;
}

void wrist_calcuations(const sensor_msgs::Joy& xbox_msg) {
	//Controls Algorithm for the two wrist motors
    //
    //
    //
	int ls_ud_detected = 0; //Integer representation of a boolean detecting whether up/down data has been
							//detected in left stick (i.e. input above 0.1 which is considered error)
	int ls_lr_detected = 0; //Integer representation of a boolean detecting whether left/right data has been
							//detected in left stick (i.e. input above 0.1 which is considered error)
    analog_write_wrist1 = 0;
    analog_write_wrist2 = 0;
    //Controls Algorithm for the Wrist Motors
    //The left analog stick is used to control both motors to achieve rotation in forward and lateral axes
    if ((abs(xbox_msg.axes[INDEX_LS_UD]) >= JOYSTICK_ERROR_THRESHOLD) || (abs(xbox_msg.axes[INDEX_LS_LR]) >= JOYSTICK_ERROR_THRESHOLD)) {
      if ((abs(xbox_msg.axes[INDEX_LS_UD]) >= JOYSTICK_ERROR_THRESHOLD)){//Detection criteria for up/down movement
          ls_ud_detected = 1;
      }
      if ((abs(xbox_msg.axes[INDEX_LS_LR]) >= JOYSTICK_ERROR_THRESHOLD)){//Detection criteria for left/right movement
          ls_lr_detected = 1;
      }
      
      //Code that details the magnitude of the voltage sent to the motors
      //If up/down or left/right movement NOT detected, will not be considered in the calculation
      //If the analog stick is positioned straight up or down, there will only be rotation about the lateral axis, but
      //if the analog stick is positioned straight left or right, there will only be rotation about the forward axis
      analog_write_wrist1 = floor(abs((DUTY_FULL)*(xbox_msg.axes[INDEX_LS_UD]*ls_ud_detected + xbox_msg.axes[INDEX_LS_LR]*ls_lr_detected)));
      analog_write_wrist2 = floor(abs((DUTY_FULL)*(xbox_msg.axes[INDEX_LS_UD]*ls_ud_detected - xbox_msg.axes[INDEX_LS_LR]*ls_lr_detected)));

      
      //Code that details the direction the motor should turn (i.e. CW or CCW)
      if ((xbox_msg.axes[INDEX_LS_UD]*ls_ud_detected + xbox_msg.axes[INDEX_LS_LR]*ls_lr_detected) >= JOYSTICK_ERROR_THRESHOLD){
          dir_wrist1_high = 1;
      }
      else {
          dir_wrist1_high = 0;
      }
      if ((xbox_msg.axes[INDEX_LS_UD]*ls_ud_detected - xbox_msg.axes[INDEX_LS_LR]*ls_lr_detected) >= JOYSTICK_ERROR_THRESHOLD){
          dir_wrist2_high = 1;
      }
      else {
          dir_wrist2_high = 0;
      }
    }
    else {
        //Default is for no output from the motors to occur. Direction output is set to LOW to conserve power.
        analog_write_wrist1 = 0;
        analog_write_wrist2 = 0;
		dir_wrist1_high = 0;
		dir_wrist1_high = 0;
    }
}
void shoulderelbow_calcuations (const sensor_msgs::Joy& xbox_msg){
	//Controls Algorithms for the shoulder and the elbow
    //
    //
    //
    //Motor controlled by the RoboClaw board require servo pulses to function
    //One servo pulse output from the Arduino is needed and the other two pins are power (5VDC) and GND
	int rs_ud_detected = 0; //Integer representation of a boolean detecting whether up/down data has been
							//detected in right stick (i.e. input above 0.1 which is considered error)
	int rs_lr_detected = 0; //Integer representation of a boolean detecting whether left/right data has been
							//detected in right stick (i.e. input above 0.1 which is considered error)
		
    analog_write_elbow = 0;
    servo_write_shoulder = SERVOPULSE_FULLSTOP;
    if ((abs(xbox_msg.axes[INDEX_RS_UD]) >= JOYSTICK_ERROR_THRESHOLD) || (abs(xbox_msg.axes[INDEX_RS_LR]) >= JOYSTICK_ERROR_THRESHOLD)) {
      if ((abs(xbox_msg.axes[INDEX_RS_UD]) >= JOYSTICK_ERROR_THRESHOLD)){//Detection criteria for up/down movement
          rs_ud_detected = 1;
      }
      if ((abs(xbox_msg.axes[INDEX_RS_LR]) >= JOYSTICK_ERROR_THRESHOLD)){//Detection criteria for left/right movement
          rs_lr_detected = 1;
      }
      
      //Code that details the magnitude of the voltage sent to the motors
      //If up/down or left/right movement NOT detected, will not be considered in the calculation
      //If the analog stick is positioned straight up or down, there will only be rotation in the elbow
      //if the analog stick is positioned straight left or right, there will only be rotation in the shoulder
      analog_write_elbow = floor(abs(DUTY_FULL_ACTUAL*(xbox_msg.axes[INDEX_RS_UD]*rs_ud_detected)));
      if (xbox_msg.buttons[INDEX_RB] == 1) {
		servo_write_shoulder = SERVOPULSE_FULLSTOP + floor(SERVOPULSE_RANGE*(xbox_msg.axes[INDEX_RS_LR]*rs_lr_detected));
	  }
	  else {
		  servo_write_shoulder = SERVOPULSE_FULLSTOP;
	  }
      
      //Code that details the direction the motor should turn (i.e. CW or CCW)
      if ((xbox_msg.axes[INDEX_RS_UD]*rs_ud_detected) >= JOYSTICK_ERROR_THRESHOLD){
          dir_elbow_high = 1;
      }
      else {
          dir_elbow_high = 0;
      }
    }
    else {
        //Default is for no output from the motors to occur. Direction output is set to LOW to conserve power.
        analog_write_elbow = 0;
		dir_elbow_high = 0;
		servo_write_shoulder = SERVOPULSE_FULLSTOP;
        
    }
}

void claw_calculations (const sensor_msgs::Joy& xbox_msg) {
	//Controls Algorithm for the Claw
    //
    //
    //
    analog_write_claw = 0;
    if ((xbox_msg.axes[INDEX_LT] < 1)) {  
      if (xbox_msg.buttons[INDEX_A] > 0) {     
			dir_claw_A = 1;
        }
        else {
            dir_claw_A = 0;
        }
        analog_write_claw = (xbox_msg.axes[INDEX_LT] - 1)*DUTY_FULL/(-2);
    }
    else {
        //If the left trigger is not pressed, no output occurs
        //Default Direction output is set to LOW to conserve power
        analog_write_claw = 0;
        dir_claw_A = 0;
    }
}

void turntable_calculations (const sensor_msgs::Joy& xbox_msg) {
	//Controls Algorithm for the TurnTable
    //
    //
    //
    //Motor controlled by the RoboClaw board require servo pulses to function
    //One servo pulse output from the Arduino is needed and the other two pins are power (5VDC) and GND
    servo_write_turntable = SERVOPULSE_FULLSTOP_TT;
    if ((xbox_msg.axes[INDEX_RT] < 1)) {   
      if (xbox_msg.buttons[INDEX_B] > 0) {
        servo_write_turntable = SERVOPULSE_FULLSTOP_TT + floor(SERVOPULSE_RANGE*(xbox_msg.axes[INDEX_RT] - 1)/(-2));
        }
        else {
            servo_write_turntable = SERVOPULSE_FULLSTOP_TT - floor(SERVOPULSE_RANGE*(xbox_msg.axes[INDEX_RT] - 1)/(-2));
        }
    }
    else {
        //If the right trigger is not pressed, no output occurs by writing the fullstop period to the motor
        servo_write_turntable = SERVOPULSE_FULLSTOP_TT;
    }  
}
