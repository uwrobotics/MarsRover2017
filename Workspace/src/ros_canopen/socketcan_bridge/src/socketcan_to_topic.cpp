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

#include <socketcan_bridge/socketcan_to_topic.h>
#include <socketcan_bridge/sensor_data.h>
#include <socketcan_interface/string.h>
#include <can_msgs/Frame.h>
#include <science_msgs/Sci_Container.h>
#include <science_msgs/Sensor.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt8.h>
#include <limits.h>
#include <linux/can.h>
#include <vector>

namespace socketcan_bridge
{

    can_msgs::Frame msg;

    SocketCANToTopic::SocketCANToTopic(boost::shared_ptr<can::DriverInterface> driver)
    {
        driver_ = driver;
        sensorData_ = SensorData(3, 20, 20); // In order: limit switch, current sensor, thermistor
        ros::Publisher* pPub = new ros::Publisher;
        if (!pPub)
        {
            ROS_ERROR("Could not allocate memory for publisher");
            return;
        }

        *pPub = nh_.advertise<can_msgs::Frame>("CAN_receiver", 10);
        topics_.push_back(pPub);
        topic_names_.push_back("CAN_receiver");
    }

    SocketCANToTopic::~SocketCANToTopic()
    {
        cleanup();
    }

    void SocketCANToTopic::cleanup()
    {
        nh_.shutdown();

        // Deallocate memory for publisher pointers
        for (int i = 0; i < topics_.size(); i++)
        {
            delete topics_[i];
            topics_[i] = NULL;
        }

        topics_.clear();
        topic_names_.clear();

        ROS_INFO("CAN receiver publishers deallocated and vectors cleared");
    }

    void SocketCANToTopic::init()
    {
        // register handler for frames and state changes.
        frame_listener_ = driver_->createMsgListener(can::CommInterface::FrameDelegate(this, &SocketCANToTopic::frameCallback));
        state_listener_ = driver_->createStateListener(can::StateInterface::StateDelegate(this, &SocketCANToTopic::stateCallback));

        cleanup();

        if (topics_.size() > 0)
        {
            topics_.clear();
        }

        if (topic_names_.size() > 0)
        {
            topic_names_.clear();
        }

        // Get parameters from .yaml file
        getParams(nh_);

        if (!publishTopic(topic_names_))
        {
            ROS_WARN("Could not publish topics");
            return;
        }
        ROS_INFO("CAN receiver initialization complete");
    }

    void SocketCANToTopic::getParams(ros::NodeHandle& nh)
    {
        nh.getParam("/receiver_list", topic_names_);
    }

    bool SocketCANToTopic::publishTopic(std::vector<std::string>& topic_list)
    {
        if (topic_list.size() == 0)
        {
            return false;
        }

        for (int i = 0; i < topic_list.size(); i++)
        {
            ros::Publisher* pPub = new ros::Publisher;

            if (!pPub)
            {
                ROS_ERROR("Could not allocate memory for publisher");
                return false;
            }

            // Advertise topic and push into vector of pointers
            // Change publisher type based on topic id
            if (i==3)
                *pPub = nh_.advertise<science_msgs::Sci_Container>(topic_list[i], 10);
            else if (i>3)
                *pPub = nh_.advertise<std_msgs::Float32MultiArray>(topic_list[i], 10);
            else
                *pPub = nh_.advertise<std_msgs::UInt8>(topic_list[i], 10);
            topics_.push_back(pPub);
        }

        return true;
    }

    void SocketCANToTopic::frameToMessage(const can::Frame& f, can_msgs::Frame& m)
    {
        m.id = f.id;
        m.dlc = f.dlc;
        m.is_error = f.is_error;
        m.is_rtr = f.is_rtr;
        m.is_extended = f.is_extended;

        for (int i = 0; i < m.dlc; i++)  // always copy all data, regardless of dlc.
        {
            m.data[i] = f.data[i];
        }
    }

    void SocketCANToTopic::frameCallback(const can::Frame& f)
    {
        // ROS_DEBUG("Message came in: %s", can::tostring(f, true).c_str());
        can::Frame frame = f;  // copy the frame first, cannot call isValid() on const.
        if (!frame.isValid())
        {
            ROS_ERROR("Invalid frame from SocketCAN: id: %#04x, length: %d, is_extended: %d, is_error: %d, is_rtr: %d",
                f.id, f.dlc, f.is_extended, f.is_error, f.is_rtr);
            return;
        }
        else
        {
            if (f.is_error)
            {
                // can::tostring cannot be used for dlc > 8 frames. It causes an crash
                // due to usage of boost::array for the data array. The should always work.
                ROS_WARN("Received error frame: %s", can::tostring(f, true).c_str());
                return;
            }
            if (f.is_rtr)
            {
                ROS_WARN("Received RTR frame: RTR frames not supported %s", can::tostring(f, true).c_str());
            }
            if (f.is_extended)
            {
                ROS_WARN("Received extended frame: extended frames not supported %s", can::tostring(f, true).c_str());
            }
        }

        can_msgs::Frame msg;
        converts the can::Frame (socketcan.h) to can_msgs::Frame (ROS msg)
        frameToMessage(frame, msg);

        msg.header.frame_id = "";  // empty frame is the de-facto standard for no frame.
        msg.header.stamp = ros::Time::now();

        int topic_idx = INT_MAX;
        bool valid_frame = true;

		//temp = msg.data;
		ROS_INFO ("dlc: %x", msg.dlc);

        ros::NodeHandle handle;
        
        ros::Publisher intPub;
        ros::Publisher sciencePub;
        ros::Publisher floatPub;

        std_msgs::UInt8 switch_msg;
        std_msgs::Float32MultiArray current_msg;
        std_msgs::Float32MultiArray thermistor_msg;
        science_msgs::Sci_Container science_msg;

        switch((msg.id)/100){
            case LIMIT_SWITCHES: // bool array, index for each switch
                ROS_INFO("Evaluating arm limit switches");
                uint8_t intResult;
                memcpy (&intResult, &msg.data, msg.dlc);
                ROS_INFO("limit switch: %x", intResult);
                switch_msg.data = intResult;
                if (msg.id%LIMIT_SWITCHES == 0)
                {
                    topic_idx = 0;
                }
                else if (msg.id%LIMIT_SWITCHES == 1)
                {
                    topic_idx = 1;
                }
                else if (msg.id%LIMIT_SWITCHES == 2)
                {
                    topic_idx = 2;
                }
                    topics_[topic_idx]->publish(switch_msg);
                break;

            case SCIENCE:
            ROS_INFO ("Evaluating science");
                if (msg.id%SCIENCE == 2)
                {
                    ROS_INFO ("Processing science limit switch data");
                    uint32_t scienceLimitSwitch;
                    memcpy (&scienceLimitSwitch, &msg.data, msg.dlc);
                    sensorData_.setScienceContainer(scienceLimitSwitch);
                }
                else // Sensor
                {
                    ROS_INFO ("Processing sensor data");
                    float sensorValue;
                    uint32_t timeStamp;
                    memcpy (&sensorValue, &msg.data, 4);
                    ROS_INFO ("Sensor value: %f", sensorValue);
                    memcpy (&timeStamp, &msg.data[4], 4);
                    ROS_INFO ("Sensor time stamp: %x", timeStamp);
                    science_msgs::Sensor sensor;
                    sensor.data = sensorValue;
                    sensor.stamp = timeStamp;
                    sensorData_.setScienceContainer(sensor, msg.id%SCIENCE);
                }
                science_msg = sensorData_.getScienceContainer();
                ROS_INFO ("Publishing science data");
                topics_[3]->publish(science_msg);
                break;

            case CURRENT_SENSORS:
                ROS_INFO("Evaluating current sensors");
                float currentResult;
                memcpy (&currentResult, &msg.data, msg.dlc);
                ROS_INFO("current data: %f", currentResult);
                sensorData_.setCurrentSensors(currentResult, msg.id%CURRENT_SENSORS);
                current_msg.data = sensorData_.getCurrentSensors();
                topics_[4]->publish(current_msg);
                break;

            case THERMISTORS:
                ROS_INFO("Evaluating thermistors");
                float thermResult;
                memcpy (&thermResult, &msg.data, msg.dlc);
                ROS_INFO("thermistor data: %f", thermResult);
                sensorData_.setThermistors(thermResult, msg.id%THERMISTORS);
                thermistor_msg.data = sensorData_.getThermistors();
                topics_[5]->publish(thermistor_msg);
                break;

            default:
                ROS_WARN("Received frame has unregistered CAN ID %x", msg.id);
                valid_frame = false;
                break;

        }
		ROS_INFO("Finished sending");
    }

    void SocketCANToTopic::stateCallback(const can::State & s)
    {
        std::string err;
        driver_->translateError(s.internal_error, err);
        if (!s.internal_error)
        {
            ROS_INFO("State: %s, asio: %s", err.c_str(), s.error_code.message().c_str());
        }
        else
        {
            ROS_ERROR("Error: %s, asio: %s", err.c_str(), s.error_code.message().c_str());
        }
    }
};  // namespace socketcan_bridge


// cansend vcan0 384#1122334455667788
// cansend vcan0 001#1111111111111111
