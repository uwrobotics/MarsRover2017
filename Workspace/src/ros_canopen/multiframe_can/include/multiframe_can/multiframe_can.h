/*********************************************************
    __  ___                   ____
   /  |/  /___ ___________   / __ \____ _   _____  _____
  / /|_/ / __ `/ ___/ ___/  / /_/ / __ \ | / / _ \/ ___/
 / /  / / /_/ / /  (__  )  / _, _/ /_/ / |/ /  __/ /
/_/  /_/\__,_/_/  /____/  /_/ |_|\____./|___/\___/_/

Copyright 2017, UW Robotics Team

@file     multiframe_can.h
@author:  Archie Lee

**********************************************************/

/*
  Every CAN frame has 8 bytes of data. This framework allows users to transmit data larger than 8 bytes over CAN.

  For multiframe publishing, ensure that your message is in a byte vector.
*/

#include <ros/ros.h>
#include <can_msgs/Frame.h>
#include <vector>

#define MAX_MSG_SIZE    64

char multiframe_CAN_publish(ros::Publisher *pub, std::vector<char>& msg, int id);

char multiframe_CAN_subscribe();
