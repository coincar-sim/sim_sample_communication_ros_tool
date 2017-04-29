#include <comm_module/comm_module.hpp>

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "comm_module_node");

    sim_sample_communication_ros_tool::CommModule comm_module(ros::NodeHandle(), ros::NodeHandle("~"));

    ros::spin();
    return 0;
}
