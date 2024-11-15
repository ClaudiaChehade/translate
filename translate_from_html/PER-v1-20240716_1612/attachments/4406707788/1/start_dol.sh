#!/bin/bash

echo "Start DoL"

export ADAS_MODE_SWITCH=1  #drving  1 #parking 2

RECOMPUTE_PLAYER_MODE="-r"
RECOMPUTE_LOG_LEVEL=1
ROUDI_MONITORING_MODE=""
MTA_GATEWAY=""
LOG_LEVEL=1
LOG_SINK=0

BUILD_DIR="$PWD"

# setup ros env and start roscore
source /opt/ros/noetic/setup.bash

#roscore&

sleep 3s

# Create dol logs folder
mkdir -p dol_logs

# export all LD_LIBRARY_PATH which required by your activity running. eg.
export LD_LIBRARY_PATH=$BUILD_DIR/lib:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$BUILD_DIR/lib_w3:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$BUILD_DIR/lib_w3/planning:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$BUILD_DIR/lib_w3/perception:$LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$BUILD_DIR/lib_aos:$LD_LIBRARY_PATH
export OPENCV_FOR_THREADS_NUM=3

# export DoL ros library This operation will be not required in W3 AOS release version MWAOS_2303.2.1
export DOL_ROS_CONVERTER_LIB=$BUILD_DIR/lib/librecompute_roudi_combined_cpp_ros_converter.so


# Start DoL target and save logs
bin/service_directory -l 4 > dol_logs/service_directory.log&


# Start your roudi here, make sure no other roudi running in your system, you can use 'ps -axu | grep roudi*' to check
# bin/FullGraph_carma_0_22_deploy_roudi_combined -lerror --monitoring-mode off 2>&1 | tee dol_logs/FullGraph_carma_0_22_deploy_roudi_combined.log&

sleep 2s

# Start your activities in DoL recompute mode and save logs, you can modify and inject your own activity here
#bin/DynamicFusionCheryWithFakeData_carma_0_22_deploy_roudi_combined_ros_gateway &

#bin/DynamicFusion_activity \
#    -m yaaac_codegen/carma_0_22_deploy/DynamicFusionCheryWithFakeData/manifests/roudi_combined_dynamic_fusion.inst \
#    -d yaaac_codegen/carma_0_22_deploy/DynamicFusionCheryWithFakeData/manifests/dynamic_fusion_deploy \
#    -p yaaac_parameter_instances \
#    -r -l 4 2>&1 | tee dol_logs/df.log &
bin/Persim_carma_0_22_deploy_roudi_combined_ros_gateway &

bin/Persim_carma_0_22_deploy_roudi_combined -lerror --monitoring-mode off&

bin/LaneFusion_activity \
    -m yaaac_codegen/carma_0_22_deploy/Persim/manifests/roudi_combined_lane_fusion.inst \
    -d yaaac_codegen/carma_0_22_deploy/Persim/manifests/lane_fusion_deploy \
    -p yaaac_parameter_instances \
    -r -l 4 2>&1 | tee dol_logs/sf.log &
    #-a 2 -s 1 -r -l 4& # 2>&1 | tee lanefusion.log&
#bin/Persim_carma_0_22_deploy_roudi_combined_ros_gateway  -lerror --monitoring-mode off&

bin/target_player -p 50001 -l 4 > dol_logs/target_player.log&

#bin/DynamicFusionCheryWithFakeData_carma_0_22_deploy_roudi_combined -lerror $ROUDI_MONITORING_MODE &
#bin/DynamicFusionCheryWithFakeData_carma_0_22_deploy_roudi_combined_ros_gateway &
#bin/LocalMsf_activity \
#    -m yaaac_codegen/carma_0_22_deploy/FullGraph/manifests/roudi_combined_local_msf.inst \
#    -d yaaac_codegen/carma_0_22_deploy/FullGraph/manifests/local_msf_deploy \
#    -p yaaac_parameter_instances \
#    -r -l 4 2>&1 | tee local_msf.log&

# It will some time to starting all activities, it depends on the activity number you run.
echo "============>> Wait for all activity starting..."
#sleep 10

 
# Start DoL host part and you will see gui, the GUI cannot be display in docker env, cause lack of QT library in docker 
bin/dol_recorder_rest_server -l 4 > dol_logs/dol_recorder_rest_server.log&
bin/dol_player_rest_server -p 5000 -l 4 > dol_logs/dol_player_rest_server.log&
bin/dol_player_gui

echo "Start DoL End"

pkill -15 -f service_directory
pkill -15 -f target_player
pkill -15 -f dol_player_gui
pkill -15 -f LaneFusion_activity
pkill -15 -f Persim_carma_0_22_deploy_roudi_combined_ros_gateway
pkill -15 -f dol_recorder_rest_server
pkill -15 -f dol_player_rest_server
pkill -15 -f Persim_carma_0_22_deploy_roudi_combined

