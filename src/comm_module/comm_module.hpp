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

#pragma once

#include <queue>
#include <random>
#include <string>
#include <dynamic_reconfigure/server.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>
#include <std_msgs/Bool.h>

#include "sim_sample_communication_ros_tool/CommModuleInterface.h"


namespace sim_sample_communication_ros_tool {

class CommModule {

    struct MsgWithTimestamp {
        ros::Time timestamp;
        boost::shared_ptr<const topic_tools::ShapeShifter> msgPtr;
    };

public:
    CommModule(ros::NodeHandle, ros::NodeHandle);

private:
    ros::Publisher pubInEx_;
    ros::Publisher pubExIn_;
    ros::Subscriber subExIn_;
    ros::Subscriber subInEx_;
    ros::Subscriber subClearQueue_;
    ros::Timer pubExInTimer_;

    dynamic_reconfigure::Server<CommModuleConfig> reconfigSrv_; // Dynamic reconfiguration service

    sim_sample_communication_ros_tool::CommModuleInterface params_;

    std::queue<MsgWithTimestamp> messageBuffer_;
    double backupTimerDuration_{10.};

    bool pubInExInitialized_ = false;
    bool pubExInInitialized_ = false;

    void subCallbackClearQueue(const std_msgs::Bool::ConstPtr &msg);
    void subCallbackExIn(ros::NodeHandle&, const boost::shared_ptr<const topic_tools::ShapeShifter>);
    void subCallbackInEx(ros::NodeHandle&, const boost::shared_ptr<const topic_tools::ShapeShifter>);
    void timerCallback(const ros::TimerEvent&);

    bool dropMessageBasedOnRandomDecision(double dropProbability);

    void reconfigureRequest(CommModuleConfig&, uint32_t);
};

} // namespace sim_sample_communication_ros_tool
