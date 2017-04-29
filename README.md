# sim_sample_communication_ros_tool
Sample communication module for a vehicle in the simulation framework.

It models characteristics of vehicle to vehicle communication, such as delay and message-drops.

## Installation
* this package is part of the simulation framework
* see simulation_management_ros_tool for installation and more details

## Usage
* started within a vehicle launchfile of the simulation_initialization_ros_tool

#### Parameters
* parameters that need to be passed to the launchfile `sample_perception.launch`:
  * **vehicle_id**: Id of the vehicle, needs to be unique within the framework
  * **vehicle_ns**: Namespace of the vehicle, needs to be unique within the framework

  * **objects_ground_truth_topic_with_ns**: Topic under which the ground truth states of the objects are received
  * **internal_communication_subns**: Subnamespace for vehicle-internal communication
  * **global_communication_ns**: Namespace for communication in-between vehicles

  * **car2x_topic1** (_optional_): Topic for car2x message type 1
  * **car2x_topic2** (_optional_): Topic for car2x message type 2
  * **car2x_topic3** (_optional_): Topic for car2x message type 3

## Contribution

* fork this repo
* use your own algorithms for modeling communication characteristics
* every message type is treated by a separate node
* ensure that
  * messages received under `/$(arg global_communication_ns)/$(arg car2x_topic1)` are forwarded under `/$(arg vehicle_ns)/$(arg internal_communication_subns)/in/$(arg car2x_topic1)` with respect to the communication characteristics model
  * messages received under `/$(arg vehicle_ns)/$(arg internal_communication_subns)/out/$(arg car2x_topic1)` are forwarded under `/$(arg global_communication_ns)/$(arg car2x_topic1)` without additional delay

## License
Contact the maintainer.
