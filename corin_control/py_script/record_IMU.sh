#!/bin/sh

## navigate to directory of bash file
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $DIR

BAGNAME=$1
FILETYPE=".bag"

echo "Logging data to ROS bag: $BAGNAME$FILETYPE"

rosbag record -O $BAGNAME$FILETYPE -b 0 /imu_LORD/data
