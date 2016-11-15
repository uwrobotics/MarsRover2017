/*
 * Copyright (c) 2016, Ivor Wanders
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <signal.h>
#include <socketcan_bridge/topic_to_socketcan.h>
#include <socketcan_interface/threading.h>
#include <socketcan_interface/string.h>
#include <string>


sig_atomic_t volatile request_shutdown = 0;

void sigIntHandler(int sig)
{
    request_shutdown = 1;
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "topic_to_socketcan_node");
    ros::NodeHandle nh_param;

    signal(SIGINT, sigIntHandler);

    std::string transmitter_interface;
    nh_param.param<std::string>("/transmitter_interface", transmitter_interface, "vcan0");

    boost::shared_ptr<can::ThreadedSocketCANInterface> driver = boost::make_shared<can::ThreadedSocketCANInterface> ();

    if (!driver->init(transmitter_interface, 0))  // initialize device at can_device, 0 for no loopback.
    {
        ROS_FATAL("Failed to initialize transmitter_interface at %s", transmitter_interface.c_str());
        return 1;
    }
    else
    {
        ROS_INFO("Successfully connected to %s.", transmitter_interface.c_str());
    }

    socketcan_bridge::TopicToSocketCAN can_transmitter(driver);
    can_transmitter.init();

    while (!request_shutdown)
    {
        ros::spinOnce();
    }

    driver->shutdown();
    driver.reset();

    ros::shutdown();

    return 0;
}
