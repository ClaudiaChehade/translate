#!/bin/bash

echo "Stop DoL"

pkill -9 -f bin/target_player
pkill -9 -f service_directory

pkill -9 -f dol_player_rest_server
pkill -9 -f dol_player_gui

pkill -9 -f FullGraph_carma_0_22_deploy_roudi_combined
pkill -9 -f Xlio_activity
pkill -9 -f LocalMsf_activity
pkill -9 -f HvmProvider_activity
pkill -9 -f OdomDriving_activity
pkill -9 -f GlobalMsfDriving_activity
pkill -9 -f dol_recorder_rest_server
pkill -9 -f LaneFusion_activity

sleep 1s

echo "Stop DoL End"





