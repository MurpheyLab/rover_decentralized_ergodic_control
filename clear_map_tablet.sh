#!/bin/bash

echo "clearing HVT/IED positions specified in the tablet for all rovers"

pushd $HOME/rover

source devel_isolated/setup.bash
source devel_isolated/rover_decentralized_ergodic_control/setup.bash

echo "clearing rover_0 HVT/IED positions"
ROS_MASTER_URI=http://localhost:11312
rosservice call /clear_map_tablet "{}"

echo "clearing rover_1 HVT/IED positions"
ROS_MASTER_URI=http://localhost:11313
rosservice call /clear_map_tablet "{}"

echo "clearing rover_2 HVT/IED positions"
ROS_MASTER_URI=http://localhost:11314
rosservice call /clear_map_tablet "{}"

echo "clearing rover_3 HVT/IED positions"
ROS_MASTER_URI=http://localhost:11315
rosservice call /clear_map_tablet "{}"

echo "clearing rover_4 HVT/IED positions"
ROS_MASTER_URI=http://localhost:11316
rosservice call /clear_map_tablet "{}"

popd
