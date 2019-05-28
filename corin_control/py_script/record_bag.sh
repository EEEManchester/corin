#!/bin/sh

## navigate to directory of bash file
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd $DIR

BAGNAME=$1
FILETYPE=".bag"

echo "Logging data to ROS bag: $BAGNAME$FILETYPE"

## Rosbag following topics: jointState, modelState, setpointState, imu, footforce
# Gazebo:
# rosbag record -O $BAGNAME$FILETYPE /corin/joint_states /gazebo/model_states /corin/setpoint_states /corin/imu/middle/data /corin/imu/offset/data /corin/contact_force

# Hardware
#rosbag record -O $BAGNAME$FILETYPE /robotis/present_joint_states /corin/setpoint_states /imu_data

rosbag record -O $BAGNAME$FILETYPE -b 0 /gazebo/model_states /corin/joint_states /corin/imu/base/data /corin/contact_state /body_r /imu_mod
