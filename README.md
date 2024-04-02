# Corin - Main Branch

Branch for active development of Corin control code. All other branches of this repo are legacy branches from previous work, and are no longer being actively maintained.

This branch is in early development stages, and thus much of the code will be preliminary.

## Running test.py
In order to run any code from this repository the correct conda environment is necessary. This can be created using the `environment.yml` files distributed with this code.

The corin manager node must be launched first in order to establish communications with the hardware. This is done using the following commands:
```
sudo bash
conda deactivate
source catkin_ws/devel/setup.bash
roslaunch corin_manager corin_manager.launch
```

Once the manager node is setup, in a new terminal execute:
```
conda activate Corin2024
source catkin_ws/devel/setup.bash
python catkin_ws/src/corin/test.py
```

## Using ROS and Conda
The conda environment `Corin2024` has been augmented using Rick Staa's [Ros Conda Wrapper](https://github.com/rickstaa/.ros_conda_wrapper) to allow it to communicate with the ROS install already present on the Corin workstation.