#pragma once

#include <string>
#include <dynamic_reconfigure/server.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>

#include "comm_module/net_sim.hpp"

#include "sim_sample_communication_ros_tool/CommModuleInterface.h"


namespace sim_sample_communication_ros_tool {

class CommModule {
public:
    CommModule(ros::NodeHandle, ros::NodeHandle);

private:
    NetSim networkSim_;
    ros::Publisher pubInEx_;
    ros::Publisher pubExIn_;
    ros::Subscriber subExIn_;
    ros::Subscriber subInEx_;

    dynamic_reconfigure::Server<CommModuleConfig> reconfigSrv_; // Dynamic reconfiguration service

    sim_sample_communication_ros_tool::CommModuleInterface params_;

    bool pubInExInitialized_ = false;
    bool pubExInInitialized_ = false;

    void subCallbackExIn(ros::NodeHandle&, const boost::shared_ptr<const topic_tools::ShapeShifter>);
    void subCallbackInEx(ros::NodeHandle&, const boost::shared_ptr<const topic_tools::ShapeShifter>);

    void timeDelay(const boost::shared_ptr<const topic_tools::ShapeShifter>);
    bool messageDrop(const boost::shared_ptr<const topic_tools::ShapeShifter>);

    void reconfigureRequest(CommModuleConfig&, uint32_t);
};

} // namespace sim_sample_communication_ros_tool
