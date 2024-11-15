#!/bin/sh

echo "Start DoL"
BUILD_DIR="$PWD"

# Create dol logs folder
mkdir -p dol_logs

export ADAS_MODE_SWITCH=1  #drving  1 #parking 2

RECOMPUTE_PLAYER_MODE="-r"
RECOMPUTE_LOG_LEVEL=1
ROUDI_MONITORING_MODE=""
MTA_GATEWAY=""
LOG_LEVEL=1
LOG_SINK=0



# setup ros env and start roscore
#source /opt/ros/noetic/setup.bash

#roscore&

sleep 3s



# export all LD_LIBRARY_PATH which required by your activity running. eg.
export LD_LIBRARY_PATH=$BUILD_DIR/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$BUILD_DIR/lib_w3:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$BUILD_DIR/lib_w3/planning:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$BUILD_DIR/lib_w3/perception:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$BUILD_DIR/lib_aos:$LD_LIBRARY_PATH
export OPENCV_FOR_THREADS_NUM=3



# Start DoL target and save logs
bin/service_directory -l 4 > dol_logs/service_directory.log&
bin/target_player -p 50018 -l 4 > dol_logs/target_player.log&
bin/dol_recorder_rest_server -p 51987 -l 4 > dol_logs/dol_recorder_rest_server.log&


sleep 2s

# Start your activities in DoL recompute mode and save logs, you can modify and inject your own activity here



# Start your roudi here, make sure no other roudi running in your system, you can use 'ps -axu | grep roudi*' to check
bin/DualOrinMaster_carma_0_22_deploy_roudi_qm -lerror --monitoring-mode off 2>&1 | tee dol_logs/DualOrinMaster_carma_0_22_deploy_roudi_combined.log&

bin/LaneFusion_activity \
    -m yaaac_codegen/carma_0_22_deploy/DualOrinMaster/manifests/roudi_qm_lane_fusion.inst \
    -d yaaac_codegen/carma_0_22_deploy/DualOrinMaster/manifests/lane_fusion_deploy \
    -p yaaac_parameter_instances \
    -a 2 -s 1 -r 2>&1 | tee dol_logs/sf.log &
    #-a 2 -s 1 -r -l 4& # 2>&1 | tee lanefusion.log&
#bin/Persim_carma_0_22_deploy_roudi_combined_ros_gateway  -lerror --monitoring-mode off&





# It will some time to starting all activities, it depends on the activity number you run.
echo "============>> Wait for all activity starting..."
sleep 1.0

 


echo "Start DoL End"



