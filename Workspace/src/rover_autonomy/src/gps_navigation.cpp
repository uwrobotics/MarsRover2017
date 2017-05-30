#include <ros/ros.h>
#include <robot_localization/navsat_conversions.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <sensor_msgs/NavSatFix.h>
#include <actionlib/client/simple_action_client.h>
#include "std_msgs/String.h"
#include <string>
#include <cmath>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <rover_autonomy/gps_coord.h>

// NOTE: Make sure the "broadcast_utm_transform" parameter is set to "true" in localization.launch

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
const int MAX_LENGTH=10;

void dividePath(double destX, double destY, double currX, double currY, std::vector<double> &divideX, std::vector<double> &divideY, int divisions){
    for (int x=1; x<=divisions; x++){
        divideX[x-1]=currX*(1-(x*1.0)/divisions) + destX * (x*1.0)/divisions;
        divideY[x-1]=currY*(1-(x*1.0)/divisions) + destY * (x*1.0)/divisions;
    }
    ROS_INFO("Generated waypoints to target");
}


void move(double latitude, double longitude){
    double UTMNorth, UTMEast;
    std::string UTMZone;
    RobotLocalization::NavsatConversions::LLtoUTM(latitude, longitude, UTMNorth, UTMEast, UTMZone); // Converts lat/long to UTM east/west
    MoveBaseClient ac("move_base", true);
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    
    // Get position of base_link wrt global utm coordinate system

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped baseLinkToUTM;  
    try{
            baseLinkToUTM = tfBuffer.lookupTransform("utm", "base_link", ros::Time(0), ros::Duration(2));
    }
    catch (tf2::TransformException ex ){
            ROS_ERROR("%s",ex.what());
    }   

    ROS_INFO("Generated transform from base_link to utm");
    double currX = baseLinkToUTM.transform.translation.x;
    double currY = baseLinkToUTM.transform.translation.y;

    int divisions = (int) ((pow ((pow((UTMEast-currX),2)+pow((UTMNorth-currY),2)),0.5))/MAX_LENGTH) + 1;

    std::vector<double> xTargets (divisions);
    std::vector<double> yTargets (divisions);   
    
    dividePath(UTMEast, UTMNorth, currX, currY, xTargets, yTargets, divisions);
    
    // Move rover sequentially to each waypoint

    ROS_INFO("Beginning movement");
    for (int x=0; x<divisions; x++){

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "utm";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = xTargets[x];
        goal.target_pose.pose.position.y = yTargets[x];
        goal.target_pose.pose.orientation.w = 1.0;


        std::stringstream ss;
        std_msgs::String msg;
        double xTemp = xTargets[x];
        double yTemp = yTargets[x];
        ss << "xTarget: " << xTemp << ", yTarget: " << yTemp << ", divisions: " << divisions;
        msg.data = ss.str();
        ROS_INFO("Sending goal: [%s]", msg.data.c_str());
        ac.sendGoal(goal);

        ac.waitForResult();
    
    }
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Movement successful");
    else
        ROS_INFO("Movement failed");
}


void receiveMessage(const rover_autonomy::gps_coord::ConstPtr& ptr){
    std::vector<double> latitudes (ptr->length);
    latitudes = ptr->latitudes;
    std::vector<double> longitudes (ptr->length);
    longitudes = ptr->longitudes;   
    for (int x=0; x<ptr->length; x++){
        move(latitudes[x], longitudes[x]);
        // Insert ball tracking stuff here
        
    


    }

}

int main(int argc, char** argv){
    ros::init(argc, argv, "gps_navigation");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/rover_autonomy/coordinates", 1000, receiveMessage); 
        
    ros::spin();
    
    return 0;
}


/* Publish rostopic on command line:

rostopic pub /rover_autonomy/coordinates rover_autonomy/gps_coord "{length: 1, latitudes:[49.901], longitudes:[8.901]}"



*/

