/*********************************************************
    __  ___                   ____
   /  |/  /___ ___________   / __ \____ _   _____  _____
  / /|_/ / __ `/ ___/ ___/  / /_/ / __ \ | / / _ \/ ___/
 / /  / / /_/ / /  (__  )  / _, _/ /_/ / |/ /  __/ /
/_/  /_/\__,_/_/  /____/  /_/ |_|\____./|___/\___/_/

Copyright 2017, UW Robotics Team

@file     multiframe_can.cpp
@author:  Archie Lee

**********************************************************/

#include <multiframe_can/multiframe_can.h>

char multiframe_CAN_publish(ros::Publisher *pub, std::vector<char>& msg, int id)
{
    int num_bytes = msg.size();
    int iterator = 0;
    int frame_number = 1;
    can::Frame frame;

    if (num_bytes > MAX_MSG_SIZE)
    {
        // error: msg too long
        return 1;
    }
    if (!pub)
    {
        // publisher doesn't exist
        return 2;
    }

    // static, these parameters won't change between frames
    frame.id = id;
    frame.is_rtr = false;
    frame.is_error = false;
    frame.is_extended = false;

    // single frame msg
    if (num_bytes <= 7)
    {
        frame.data[0] = (num_bytes & 0xF);
        for (int i = 1; i < 7; i++)
        {
           frame.data[i] = msg[i-1];
        }

        pub->publish(frame);
    }
    // multiframe msg
    else
    {
        int start;
        while (iterator < num_bytes)
        {
            // first frame
            if (frame_number == 1)
            {
                frame.data[0] = 1 << 4;
                frame.data[0] |= ((num_bytes >> 8) & 0xF);
                frame.data[1] = (num_bytes & 0xFF);

                start = 2;
            }
            // consecutive frames
            else
            {
                frame.data[0] = 2 << 4;
                frame.data[0] |= (frame_number & 0xF);

                start = 1;
            }

            // populate data
            for (int i = start; i < frame.dlc; i++)
            {
                frame.data[i] = msg[iterator];
            }

            // move iterator forward
            if (iterator + 8 < num_bytes)
            {
                frame.dlc = 8;
                iterator += 8;
            }
            else
            {
                frame.dlc = num_bytes - iterator - 1;
                iterator = num_bytes;
            }

            // publish frame and increment frame number
            pub->publish(frame);
            frame_number++;
        }
    }

    return 0;
}
