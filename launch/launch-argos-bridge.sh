#!/bin/bash

# The purpose of this script is to create a launch file for a multi-robot
# system by replicating a "group" tag "n" times, then executing the resulting
# launch file.

# The number of robots.  This should match the 'quantity' value in the argos world file (e.g. argos_worlds/demo.argos).

n=320

LAUNCH_FILE=/tmp/argos_interface.launch.py
CONTROLLER_DIR=/home/sindiso/ros2_dev/src/flocking
ARGOS_CONFIG_DIR=$CONTROLLER_DIR/launch/flocking320.argos

echo "import os" > $LAUNCH_FILE
echo -e "import pathlib" >> $LAUNCH_FILE
echo -e "import launch" >> $LAUNCH_FILE
echo -e "import yaml" >> $LAUNCH_FILE
echo -e "from launch import LaunchDescription" >> $LAUNCH_FILE
echo -e "from launch_ros.actions import Node" >> $LAUNCH_FILE

echo -e "from ament_index_python.packages import get_package_share_directory" >> $LAUNCH_FILE

echo -e "def generate_launch_description():" >> $LAUNCH_FILE
echo -e "\tconfig_dir = os.path.join('/home/sindiso/ros2_dev/src/footbot_flocking', 'config')" >> $LAUNCH_FILE
echo -e "\tparam_config = os.path.join(config_dir, \"config.yaml\")" >> $LAUNCH_FILE
echo -e "\twith open(param_config, 'r') as f:" >> $LAUNCH_FILE
echo -e "\t\tparams = yaml.safe_load(f)[\"footbot_flocking_controller\"][\"ros__parameters\"]" >> $LAUNCH_FILE
    
echo -e "\tld = LaunchDescription()" >> $LAUNCH_FILE

for ((i=0; i<n; i++)); do
    namespace="bot$i"
    echo -e "\t$namespace = Node(package=\"argos3_ros2_bridge\", executable=\"flocking\", name=\"flocking\", output=\"screen\", namespace=\"$namespace\", parameters=[params])" >> $LAUNCH_FILE
    echo -e "\tld.add_action($namespace)" >> $LAUNCH_FILE
done >> $LAUNCH_FILE
echo -e "\treturn ld" >> $LAUNCH_FILE

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/argos3:/home/sindiso/ros2_dev/install/argos3_ros2_bridge/lib
export ARGOS_PLUGIN_PATH=/home/sindiso/ros2_dev/install/argos3_ros2_bridge/lib/
# keep topics local to computer
export ROS_LOCALHOST_ONLY=1

argos3 -c $ARGOS_CONFIG_DIR

#ros2 launch $LAUNCH_FILE 

