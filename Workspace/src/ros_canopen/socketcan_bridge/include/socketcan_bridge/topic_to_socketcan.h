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

#ifndef SOCKETCAN_BRIDGE_TOPIC_TO_SOCKETCAN_H
#define SOCKETCAN_BRIDGE_TOPIC_TO_SOCKETCAN_H

#include <socketcan_interface/socketcan.h>
#include <can_msgs/Frame.h>
#include <ros/ros.h>
#include <vector>
#include <string>

namespace socketcan_bridge
{
    class TopicToSocketCAN
    {
        public:
            TopicToSocketCAN(boost::shared_ptr<can::DriverInterface> driver);
            void init();

        private:
            ros::NodeHandle nh_;
            ros::Subscriber can_topic_;
            boost::shared_ptr<can::DriverInterface> driver_;

            can::StateInterface::StateListener::Ptr state_listener_;

            void getParams(ros::NodeHandle& nh);
            void messageToFrame(const can_msgs::Frame& m, can::Frame& f);
            void msgCallback(const can_msgs::Frame::ConstPtr& msg);
            void stateCallback(const can::State & s);
    };
};  // namespace socketcan_bridge


#endif  // SOCKETCAN_BRIDGE_TOPIC_TO_SOCKETCAN_H
