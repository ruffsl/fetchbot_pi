#!/bin/sh

# ././devel/.private/mars_ros/lib/mars_ros/micarray_recorder ls -1* raw
TRACK_DIR=/home/pi/workspaces/mars_ros/build/mars/devel/lib/mars/demo
CONFIG=/home/pi/workspaces/mars_ros/src/mars/config/matrixcreator.cfg
# PRE_DIR=/tmp/fetchbot/
PRE_DIR=/home/pi/workspaces/mars_ros/src/

# ./build/mars/devel/lib/mars/demo -i src/mars_ros/mars_ros/src/channel_all.raw -c src/mars/config/matrixcreator.cfg -v
# 2.7s
# echo $
trackarg=$TRACK_DIR" -i "$PRE_DIR"channel_all.raw -c "$CONFIG" -v > "$PRE_DIR"tracking.out" 
eval $trackarg
