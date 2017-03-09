#!/bin/sh

# ././devel/.private/mars_ros/lib/mars_ros/micarray_recorder ls -1* raw
REC_DIR=/home/pi/workspaces/matrix-creator-hal/build/demos/
PRE_DIR=/tmp/fetchbot/

# 3 seconds
eval $REC_DIR"micarray_recorder ls -1 *raw"

# 1.25 seconds
for i in `seq 1 8`; do
  soxarg="sox -r 16000 -c 1 -e signed -c 1 -e signed -b 16 "$PRE_DIR
  # soxarg+=
  soxarg=$soxarg"mic_16000_s16le_channel_"$i
  soxarg=$soxarg".raw -r 48000 "$PRE_DIR"channel_"$i".raw"
  $soxarg
  #  echo $worker
   #$worker
  #statements
done

# 0.1 seconds
soxarg="sox -M "
for i in `seq 1 8`; do
  soxarg=$soxarg"-r 48000 -e signed -b 16 "$PRE_DIR"channel_"$i".raw "
done
soxarg=$soxarg"-r 48000 -e signed -b 16 "$PRE_DIR"channel_all.raw"

eval $soxarg
