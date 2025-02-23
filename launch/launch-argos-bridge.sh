#!/bin/bash

# The purpose of this script is to create a launch file for a multi-robot
# system by replicating a "group" tag "n" times, then executing the resulting
# launch file.

# The number of robots.  This should match the 'quantity' value in the argos world file (e.g. argos_worlds/demo.argos).


LAUNCH_FILE=/tmp/argos_interface.launch.py
CONTROLLER_DIR=/home/sindiso/ros2_dev/src/flocking
ARGOS_CONFIG_DIR=$CONTROLLER_DIR/launch/flocking320.argos


export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/argos3:/home/sindiso/ros2_dev/install/argos3_ros2_bridge/lib
export ARGOS_PLUGIN_PATH=/home/sindiso/ros2_dev/install/argos3_ros2_bridge/lib/
# keep topics local to computer
#export ROS_LOCALHOST_ONLY=1

RMW_IMPLEMENTATION=rmw_cyclonedds_cpp argos3 -c $ARGOS_CONFIG_DIR -z

