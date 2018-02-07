#pragma once

#include <queue>
#include <random>
#include <string>
#include <dynamic_reconfigure/server.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>

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
    ros::Timer pubExInTimer_;

    dynamic_reconfigure::Server<CommModuleConfig> reconfigSrv_; // Dynamic reconfiguration service

    sim_sample_communication_ros_tool::CommModuleInterface params_;

    std::queue<MsgWithTimestamp> messageBuffer_;
    double backupTimerDuration_{10.};

    bool pubInExInitialized_ = false;
    bool pubExInInitialized_ = false;

    void subCallbackExIn(ros::NodeHandle&, const boost::shared_ptr<const topic_tools::ShapeShifter>);
    void subCallbackInEx(ros::NodeHandle&, const boost::shared_ptr<const topic_tools::ShapeShifter>);
    void timerCallback(const ros::TimerEvent&);

    bool dropMessageBasedOnRandomDecision(double dropProbability);

    void reconfigureRequest(CommModuleConfig&, uint32_t);
};

} // namespace sim_sample_communication_ros_tool
