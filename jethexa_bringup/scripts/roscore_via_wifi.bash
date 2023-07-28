#!/bin/bash

IP=$(ip a|grep wlan0|grep 'inet'|awk '{print $2}')
MY_IP=${IP%/*}
MASTER_IP="${IP%.*}.1"

export ROS_HOSTNAME=$MY_IP
export ROS_MASTER_URI=http://$MASTER_IP:11311

printf "ROS_HOSTNAME=\e[32m$ROS_HOSTNAME\e[0m\n"
printf "ROS_MASTER_URI=\e[32m$ROS_MASTER_URI\e[0m\n"

