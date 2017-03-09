#!/bin/sh

# ././devel/.private/mars_ros/lib/mars_ros/micarray_recorder ls -1* raw
TRACK_DIR=/home/pi/workspaces/mars_ros/build/mars/devel/lib/mars/demo
CONFIG=/home/pi/workspaces/mars_ros/src/mars/config/matrixcreator.cfg
# PRE_DIR=/tmp/fetchbot/
PRE_DIR=/home/pi/workspaces/mars_ros/src/

# ./build/mars/devel/lib/mars/demo -i src/mars_ros/mars_ros/src/channel_all.raw -c src/mars/config/matrixcreator.cfg -v
# 2.7s
# echo $
xchar=$(tail -18 out | head -1 | cut -c 8-14)
ychar=$(tail -18 out | head -1 | cut -c 17-21)
zchar=$(tail -18 out | head -1 | cut -c 23-)
