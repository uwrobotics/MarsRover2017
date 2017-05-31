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
#include <visualization_msgs/Marker.h>

// NOTE: Make sure the "broadcast_utm_transform" parameter is set to "true" in localization.launch

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
const int MAX_LENGTH=5;
ros::Publisher waypoints_viz_pub, divisions_viz_pub;

void dividePath(double destX, double destY, double currX, double currY, std::vector<double> &divideX, std::vector<double> &divideY, int divisions){
    for (int x=1; x<=divisions; x++){
        divideX[x-1]=currX*(1-(x*1.0)/divisions) + destX * (x*1.0)/divisions;
        divideY[x-1]=currY*(1-(x*1.0)/divisions) + destY * (x*1.0)/divisions;
    }
    ROS_INFO("Generated waypoints to target");
}


void move(double UTMEast, double UTMNorth){
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

    ROS_WARN("Moving to next waypoint... (%d divisions)", divisions);

    visualization_msgs::Marker divisions_viz;
    divisions_viz.header.frame_id = "utm";
    divisions_viz.id = 0;
    divisions_viz.type = visualization_msgs::Marker::SPHERE_LIST;
    divisions_viz.action = visualization_msgs::Marker::ADD;
    divisions_viz.ns = "goal_waypoints";
    divisions_viz.scale.x = 0.3;
    divisions_viz.scale.y = 0.3;
    divisions_viz.scale.z = 0.3;
    divisions_viz.color.r = 1.0;
    divisions_viz.color.g = 0.0;
    divisions_viz.color.b = 0.0;
    divisions_viz.color.a = 1.0;

    // publish the divisions list
    for (int i = 0; i < divisions; i++) {
        geometry_msgs::Point pt_viz;
        pt_viz.x = xTargets[i];
        pt_viz.y = yTargets[i];
        pt_viz.z = 0;
        divisions_viz.points.push_back(pt_viz);
    }

    divisions_viz_pub.publish(divisions_viz);

    for (int i=0; i < divisions && ros::ok(); i++) {

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "utm";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = xTargets[i];
        goal.target_pose.pose.position.y = yTargets[i];
        goal.target_pose.pose.orientation.w = 1.0;


        std::stringstream ss;
        std_msgs::String msg;
        double xTemp = xTargets[i];
        double yTemp = yTargets[i];
        ss << "xTarget: " << xTemp << ", yTarget: " << yTemp << ", divisions: " << divisions;
        ROS_INFO("Sending goal (x,y) = (%.2f, %.2f)", xTemp, yTemp);
        msg.data = ss.str();
        ac.sendGoal(goal);

        ac.waitForResult();
    }
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_WARN("Completed waypoint");
    else
        ROS_ERROR("Failed to complete waypoint");
}


void receiveMessage(const rover_autonomy::gps_coord::ConstPtr& ptr){
    std::vector<double> UTMEasts(ptr->length);
    std::vector<double> UTMNorths(ptr->length);

    visualization_msgs::Marker waypoints_viz;
    waypoints_viz.header.frame_id = "utm";
    waypoints_viz.id = 1;
    waypoints_viz.type = visualization_msgs::Marker::SPHERE_LIST;
    waypoints_viz.action = visualization_msgs::Marker::ADD;
    waypoints_viz.ns = "goal_waypoints";
    waypoints_viz.scale.x = 1;
    waypoints_viz.scale.y = 1;
    waypoints_viz.scale.z = 1;
    waypoints_viz.color.r = 0.0;
    waypoints_viz.color.g = 1.0;
    waypoints_viz.color.b = 0.0;
    waypoints_viz.color.a = 1.0;

    for (int i = 0; i < ptr->length; i++) {
        std::string UTMZone;
        RobotLocalization::NavsatConversions::LLtoUTM(ptr->latitudes[i], ptr->longitudes[i],
            UTMNorths[i], UTMEasts[i], UTMZone); // Converts lat/long to UTM east/west

        geometry_msgs::Point pt_viz;
        pt_viz.x = UTMEasts[i];
        pt_viz.y = UTMNorths[i];
        pt_viz.z = 0;
        waypoints_viz.points.push_back(pt_viz);
    }

    waypoints_viz_pub.publish(waypoints_viz);

    for (int i=0; i<ptr->length && ros::ok(); i++){
        move(UTMEasts[i], UTMNorths[i]);

        // tennis ball detection
        for (angle = 0; angle < 360; angles+=15) {
            // tell gimbal to go to an angle
            // grab a frame
            // get the tennis ball position
            
            
        }

        // find the tennis ball

    }

}

int main(int argc, char** argv){
    ros::init(argc, argv, "gps_navigation");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/rover_autonomy/coordinates", 1000, receiveMessage);
    waypoints_viz_pub = n.advertise<visualization_msgs::Marker>("waypoints_viz", 1, true);
    divisions_viz_pub = n.advertise<visualization_msgs::Marker>("divisions_viz", 1, true);

    ros::spin();

    return 0;
}


/* Publish rostopic on command line:

rostopic pub /rover_autonomy/coordinates rover_autonomy/gps_coord "{length: 1, latitudes:[49.901], longitudes:[8.901]}"

*/

