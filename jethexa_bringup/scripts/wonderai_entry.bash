#!/bin/bash
source $HOME/.hiwonderrc

# Whatever ROS_HOSTNAME and ROS_MASTER_URI are set
# change them to localhost to avoid the situation that the app is out of service when it is switched to LAN mode due to the changed IP address
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
export DISPLAY=:0.0
exec "$@"
