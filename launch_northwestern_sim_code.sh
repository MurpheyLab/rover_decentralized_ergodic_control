#!/bin/bash

echo "launching Northwestern ROS Code..."

#source localDevice.properties
source sim.properties

pushd $HOME/rover

source devel_isolated/setup.bash
source devel_isolated/rover_decentralized_ergodic_control/setup.bash

# turn on sim for the first rover for deugging
echo "launching rover_0"
ROS_MASTER_URI=http://localhost:11312
roslaunch rover_decentralized_ergodic_control ergodic_control.launch agent_id:=rover_0 show_in_rviz:=True> rover_0.txt 2>&1 &

# launch the other rovers without debugging
rosPort=11313
for i in $(seq 1 $(($numRovers-1))); do

  echo "launching rover_$i"
  ROS_MASTER_URI=http://localhost:$rosPort
  roslaunch rover_decentralized_ergodic_control ergodic_control.launch agent_id:=rover_$i > rover_$i.txt 2>&1 &

  let rosPort=$(($rosPort+1))

done


# Debugging tools / plotting comment out if not wanted
#ROS_MASTER_URI=http://localhost:11312
#echo "creating reconstruction and target_distribution plot"
#roslaunch rover_decentralized_ergodic_control visualize_swarm_paths.launch agent_id:=rover_0 > plotting_ouput_text.txt 2>&1 &

popd
