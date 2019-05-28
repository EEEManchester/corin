#!/bin/sh

## navigate to directory
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $DIR

echo "Unpacking ROS bag: $1"

BAGNAME=$1
FILETYPE=".bag"

# Hardware
rostopic echo -b $BAGNAME$FILETYPE -p /robotis/present_joint_states > ./csv/hw_joint_cs.csv
rostopic echo -b $BAGNAME$FILETYPE -p /imu/data > ./csv/hw_imu.csv
