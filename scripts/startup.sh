#!/bin/bash

source ~/.bashrc

roslaunch srcsim finals.launch
#XAUTHORITY=/tmp/.docker.xauth2 && sudo x11vnc -create -forever -passwd 1234 -repeat -users whrl &

#change the ros variables and start another ros core
# export ROS_MASTER_URI=http://192.168.2.1:11311
# export ROS_IP=192.168.2.10

#roscore -p 8000 &
#rosrun master_discovery_fkie master_discovery _mcast_group:=224.0.0.1 _rpc_port:=11611 &
#rosrun master_sync_fkie master_sync &

#while true; do
#    sleep 50
#done
