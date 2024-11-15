#!/bin/bash

echo "Start DoL"

#export ADAS_MODE_SWITCH=1  #drving  1 #parking 2

RECOMPUTE_PLAYER_MODE="-r"
RECOMPUTE_LOG_LEVEL=1
ROUDI_MONITORING_MODE=""
MTA_GATEWAY=""
LOG_LEVEL=1
LOG_SINK=0

BUILD_DIR="$PWD"

# setup ros env and start roscore
source /opt/ros/noetic/setup.bash

roscore&

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
#export DOL_ROS_CONVERTER_LIB=$BUILD_DIR/lib/librecompute_roudi_combined_cpp_ros_converter.so



# Start your roudi here, make sure no other roudi running in your system, you can use 'ps -axu | grep roudi*' to check
# bin/FullGraph_carma_0_22_deploy_roudi_combined -lerror --monitoring-mode off 2>&1 | tee dol_logs/FullGraph_carma_0_22_deploy_roudi_combined.log&

sleep 2s


# Start DoL host part and you will see gui, the GUI cannot be display in docker env, cause lack of QT library in docker 
bin/dol_recorder_rest_server -l 4 > dol_logs/dol_recorder_rest_server.log&
bin/dol_player_rest_server -p 5000 -l 4 > dol_logs/dol_player_rest_server.log&
bin/dol_player_gui

echo "Start DoL End"




