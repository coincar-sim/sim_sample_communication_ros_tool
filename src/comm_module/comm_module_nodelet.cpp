#include <comm_module/comm_module.hpp>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace sim_sample_communication_ros_tool {

class ReceiverNodelet : public nodelet::Nodelet {

    virtual void onInit();
    boost::shared_ptr<CommModule> m_;
};

void ReceiverNodelet::onInit() {
    m_.reset(new CommModule(getNodeHandle(), getPrivateNodeHandle()));
}

} // namespace sim_sample_communication_ros_tool

PLUGINLIB_DECLARE_CLASS(sim_sample_communication_ros_tool, ReceiverNodelet, sim_sample_communication_ros_tool::ReceiverNodelet, nodelet::Nodelet);
