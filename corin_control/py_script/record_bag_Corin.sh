#!/bin/sh

## navigate to directory of bash file
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $DIR

BAGNAME=$1
FILETYPE=".bag"

echo "Logging data to ROS bag: $BAGNAME$FILETYPE"

rosbag record -O $BAGNAME$FILETYPE -b 0 /robotis/present_joint_states /imu/data /imu_LORD/data /force_vector_0 /force_vector_1 /force_vector_2 /force_vector_3 /force_vector_4 /force_vector_5
