#!/bin/bash

BAGFILE=$1
MAPNAME="peccioli@Home"

if [ -n "$2" ]; then
  MAPNAME="$2"
fi

echo $MAPNAME

roslaunch rockin_logger replay.launch bag_file:=$BAGFILE map_name:=$MAPNAME
 
