#!/bin/bash

echo "clearing HVT/IED positions specified in STOMP for all rovers"

pushd $HOME/rover

source devel_isolated/setup.bash
source devel_isolated/rover_decentralized_ergodic_control/setup.bash

echo "clearing rover_0 HVT/IED positions"
ROS_MASTER_URI=http://localhost:11312
rosservice call /clear_map_tablet "{}"

popd
