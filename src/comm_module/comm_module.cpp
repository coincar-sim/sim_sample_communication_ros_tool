/*
 * Copyright (c) 2017
 * FZI Forschungszentrum Informatik, Karlsruhe, Germany (www.fzi.de)
 * KIT, Institute of Measurement and Control, Karlsruhe, Germany (www.mrt.kit.edu)
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "comm_module.hpp"

using namespace topic_tools;

namespace sim_sample_communication_ros_tool {

CommModule::CommModule(ros::NodeHandle node_handle, ros::NodeHandle private_node_handle)
        : reconfigSrv_{private_node_handle}, params_{private_node_handle} {

    /**
     * Initialization
     */
    params_.fromParamServer();

    /**
     * Set up dynamic reconfiguration
     */
    reconfigSrv_.setCallback(boost::bind(&CommModule::reconfigureRequest, this, _1, _2));

    /**
     * Publishers & subscriber
     */
    // Subscriber for generic messages from random external topics. Networksimulation possible.
    subExIn_ = node_handle.subscribe<ShapeShifter>(params_.communication_external_topic,
                                                   params_.msg_queue_size,
                                                   boost::bind(&CommModule::subCallbackExIn, this, node_handle, _1));

    // Subscriber for internal to external communication. No networksimulation required.
    subInEx_ = node_handle.subscribe<ShapeShifter>(params_.communication_internal_out_topic,
                                                   params_.msg_queue_size,
                                                   boost::bind(&CommModule::subCallbackInEx, this, node_handle, _1));

        subClearQueue_ = node_handle.subscribe<std_msgs::Bool>(params_.clear_queue_topic,
                                                               params_.msg_queue_size,
                                                               &CommModule::subCallbackClearQueue, this);

        pubExInTimer_ = node_handle.createTimer(ros::Duration(backupTimerDuration_), &CommModule::timerCallback, this);
    }

    void CommModule::subCallbackClearQueue(const std_msgs::Bool::ConstPtr &msg){
        if(msg->data){
            while(!messageBuffer_.empty()){
                messageBuffer_.pop();
            }
        }
    }


void CommModule::subCallbackExIn(ros::NodeHandle& nh, const boost::shared_ptr<const ShapeShifter> msg) {

    if (!pubExInInitialized_) {
        // initialize publisher with first message
        pubExIn_ = msg->advertise(nh, params_.communication_internal_in_topic, params_.msg_queue_size, false);
        pubExInInitialized_ = true;
    }

    MsgWithTimestamp msgWT;
    msgWT.msgPtr = msg;
    msgWT.timestamp = ros::Time::now();
    // todo: filter messages from ego object
    messageBuffer_.push(msgWT);

    ROS_DEBUG("Received message at t= %s, queuesize=%s",
              std::to_string(ros::Time::now().toSec()).c_str(),
              std::to_string(messageBuffer_.size()).c_str());

    if (messageBuffer_.size() == 1) {
        timerCallback(ros::TimerEvent());
    }
}

/**
 * This callback is called when data is relayed from a vehicle-internal topic to an external topic.
 * No Netwirk-Simulation required.
 */
void CommModule::subCallbackInEx(ros::NodeHandle& nh, const boost::shared_ptr<const ShapeShifter> msg) {

    if (!pubInExInitialized_) {
        // initialize publisher with first message
        pubInEx_ = msg->advertise(nh, params_.communication_external_topic, params_.msg_queue_size, false);
        pubInExInitialized_ = true;
    }

    pubInEx_.publish(*msg);
}

/**
 *  External -> internal communication.
 *  realizes time-delay and message_drop
 */
void CommModule::timerCallback(const ros::TimerEvent& e) {

    if (messageBuffer_.empty()) {
        ROS_DEBUG("Waiting for message at     t= %s", std::to_string(ros::Time::now().toSec()).c_str());
        pubExInTimer_.setPeriod(ros::Duration(backupTimerDuration_));
        return;
    }

    MsgWithTimestamp msgWT = messageBuffer_.front();

    double delay = (ros::Time::now() - msgWT.timestamp).toSec();
    double delayDifference = params_.time_delay - delay;

    // determine whether to sent the message or not (regarding time delay)
    if (delayDifference < 0) {
        ROS_DEBUG("Attempt to sent message at t= %s, queuesize=%s, delay=%s",
                  std::to_string(ros::Time::now().toSec()).c_str(),
                  std::to_string(messageBuffer_.size()).c_str(),
                  std::to_string(delay).c_str());

        // determine whether to sent the message or not (regarding message drop)
        if (!dropMessageBasedOnRandomDecision(params_.drop_probability)) {
            pubExIn_.publish(*msgWT.msgPtr);
            ROS_DEBUG("Sent message at t= %s, queuesize=%s, delay=%s",
                      std::to_string(ros::Time::now().toSec()).c_str(),
                      std::to_string(messageBuffer_.size()).c_str(),
                      std::to_string(delay).c_str());
        }

        // remove the message from the buffer
        messageBuffer_.pop();

        // check the timer callback for the next message
        timerCallback(e);

    } else {
        // wait as the message is only sent in the next timer callback
        pubExInTimer_.setPeriod(ros::Duration(delayDifference));
    }
}

/**
 *  Randomly determine whether a message will be dropped
 */
bool CommModule::dropMessageBasedOnRandomDecision(double dropProbability) {

    if (dropProbability < 10e-6) {
        ROS_DEBUG("Drop: not dropping as dropProbability < 10e-6");
        return false;
    }

    double value = rand() / static_cast<double>(RAND_MAX);

    if (value < dropProbability) {
        ROS_DEBUG("Drop: rand()=%s , drop_margin=%s -> dropping",
                  std::to_string(value).c_str(),
                  std::to_string(dropProbability).c_str());
        return true;
    } else {
        ROS_DEBUG("Drop: rand()=%s , drop_margin=%s -> not dropping",
                  std::to_string(value).c_str(),
                  std::to_string(dropProbability).c_str());
        return false;
    }
}


/**
 * This callback is called whenever a change was made in the dynamic_reconfigure window
 */
void CommModule::reconfigureRequest(CommModuleConfig& config, uint32_t level) {
    params_.fromConfig(config);
    timerCallback(ros::TimerEvent());
}


} // namespace sim_sample_communication_ros_tool
