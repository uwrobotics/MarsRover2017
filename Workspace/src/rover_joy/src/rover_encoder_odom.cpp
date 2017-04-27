#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "roboteq_msgs/Feedback.h"


double enc_vel_left = 0;
double enc_vel_right = 0;

void left_feeback_callback(roboteq_msgs::Feedback &msg) {

}

void right_feedback_callback(roboteq_msgs::Feedback &msg) {

}


int main(int argc, char** argv){
  ros::init(argc, argv, "rover_encoder_odom");

  ros::NodeHandle n;

  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1);
  ros::Subscriber left_feeback_sub = n.subscribe("/left/feedback", 1, left_feeback_callback);
  ros::Subscriber right_feeback_sub = n.subscribe("/right/feedback", 1, right_feedback_callback);

  double pos_x = 0.0;
  double pos_y = 0.0;
  double pos_th = 0.0;

  double vx, vy, vth;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate loop_rate(30);
  while(ros::ok()){

    loop_rate.sleep(); //Maintain the loop rate
    ros::spinOnce();   //Check for new messages

    current_time = ros::Time::now();

    vx = 0;
    vy = 0;
    vth = 0;

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
  }
}