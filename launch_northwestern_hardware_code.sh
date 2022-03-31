#!/bin/bash

echo "launching Northwestern ROS Code..."

pushd $HOME/rover

source devel_isolated/setup.bash
source devel_isolated/rover_decentralized_ergodic_control/setup.bash

echo "launching nu code on rover"

# ROS_MASTER_URI=http://tx2-f84638:11311
# ROS_MASTER_URI=http://tx2-f8463b:11311

#roslaunch rover_decentralized_ergodic_control real_hardware_ergodic_control.launch agent_id:=tx2-f84638 > rover_0.txt 2>&1 &
roslaunch rover_decentralized_ergodic_control real_hardware_ergodic_control.launch agent_id:=tx2-f8463b > rover_0.txt 2>&1 &

popd
