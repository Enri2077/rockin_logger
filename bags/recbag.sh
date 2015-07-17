#!/bin/bash

TEAMNAME="adams"

BENCHMARK="TBMH1"	# {TBM|FBM}{H|W}{1..3}

if [ -n "$1" ]; then
  TEAMNAME="$1"
fi

if [ -n "$2" ]; then
  BENCHMARK="$2"
fi

DATE=`date +%0Y%0m%0d%0k%0M`

xterm -e "roslaunch rockin_logger rockin_logger.launch" &

rosbag record /tf /rockin/${TEAMNAME}/robot_pose /rockin/${TEAMNAME}/marker_pose /rockin/${TEAMNAME}/scan_0 /rockin/${TEAMNAME}/scan_1 /rockin/${TEAMNAME}/image \
              /rockin/${TEAMNAME}/pointcloud /rockin/${TEAMNAME}/audio /rockin/${TEAMNAME}/command \
              -O "${BENCHMARK}_${DATE}_${TEAMNAME}.bag" 
