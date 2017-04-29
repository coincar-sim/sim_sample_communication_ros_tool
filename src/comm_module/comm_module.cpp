#include <comm_module/comm_module.hpp>

using namespace topic_tools;
using namespace automated_driving_msgs;

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
    subExIn_ = node_handle.subscribe<ShapeShifter>(
        params_.communication_external_topic,
        params_.msg_queue_size,
        boost::bind(&CommModule::subCallbackExIn, this, node_handle, _1));

    // Subscriber for internal to external communication. No networksimulation required.
    subInEx_ = node_handle.subscribe<ShapeShifter>(
        params_.communication_internal_out_topic,
        params_.msg_queue_size,
        boost::bind(&CommModule::subCallbackInEx, this, node_handle, _1));
}

/**
 *  External -> internal communication.
 *  realizes time-delay and message_drop
 */
void CommModule::subCallbackExIn(ros::NodeHandle& nh,
                                 const boost::shared_ptr<const ShapeShifter> msg) {
    pubExIn_ =
        msg->advertise(nh, params_.communication_internal_in_topic, params_.msg_queue_size, false);

    if (params_.time_delay > 0) {
        timeDelay(msg);
    }

    messageDrop(msg);
}

/**
 * This callback is called when data is relayed from a vehicle-internal topic to an external topic.
 * No Netwirk-Simulation required.
 */

void CommModule::subCallbackInEx(ros::NodeHandle& nh,
                                 const boost::shared_ptr<const ShapeShifter> msg) {
    pubInEx_ =
        msg->advertise(nh, params_.communication_external_topic, params_.msg_queue_size, false);
}


/**
  * This callback is called whenever a change was made in the dynamic_reconfigure window
*/
void CommModule::reconfigureRequest(CommModuleConfig& config, uint32_t level) {
    params_.fromConfig(config);
}


/**
 * realizes time-delay
 * \param msg Message that shall be delayed
 */
void CommModule::timeDelay(const boost::shared_ptr<const ShapeShifter> msg) {
    if (params_.time_delay != 4) {
        networkSim_.setStateSender(params_.time_delay);
        double delay = networkSim_.getDelayTime();

        ROS_DEBUG("Delaying message by %s seconds", std::to_string(delay).c_str());
        ros::Duration(delay).sleep();
        ROS_DEBUG("Delayed message by %s seconds", std::to_string(delay).c_str());
    }
}

/**
 * realizes message-drop
 * \param msg Message
 * \return True: Message dropped; False: No Message Drop.
 */

bool CommModule::messageDrop(const boost::shared_ptr<const ShapeShifter> msg) {
    if (params_.time_delay == 4) {
        ROS_DEBUG("Message dropped");
        return true;
    } else {
        networkSim_.setDropProbability(params_.drop_probability);
        if (!networkSim_.dropMessage()) {
            pubExIn_.publish(*msg);
            ROS_DEBUG("Message relayed");
            return false;
        } else {
            ROS_DEBUG("Message dropped");
            return true;
        }
    }
}

} // namespace sim_sample_communication_ros_tool
