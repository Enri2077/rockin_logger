#!/bin/bash

TEAMNAME="lurch"

BENCHMARK="TBMH1" # {TBM|FBM}{H|W}{1..3}

if [ -n "$1" ]; then
  TEAMNAME="$1"
fi

if [ -n "$2" ]; then
  BENCHMARK="$2"
fi

DATE=`date +%Y%m%d%k%M`

xterm -e "roslaunch rockin_logger rockin_logger.launch" &

rosbag record /tf /rockin/robot_pose /rockin/marker_pose /rockin/scan_0 /rockin/scan_1 /rockin/image \
              /rockin/image /rockin/pointcloud /rockin/audio /rockin/command \
              -O "${BENCHMARK}_${DATE}_${TEAMNAME}.bag" 
