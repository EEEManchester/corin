# corin

Development branch for state estimation. Research from this branch was published in the journal 

    Hassan H. Khalili, Wei Cheah, Tomas B. Garcia-Nathan, Joaquin Carrasco, Simon Watson, and Barry Lennox. Tuning and sensitivity analysis of a hexapod state estimator, Robotics and Autonomous Systems, Volume 129, 2020.

## State Estimator Simulation

Refer to `Corin_EKF_0.1.pdf` for the theory behind the state estimator implementation. Further information can also be found in the journal paper above.

Firstly, the state estimator was tested on the Gazebo model. The simulation sample rate was set to 10kHz in `corin/corin_gazebo/worlds/empty.world`:

    <max_step_size>0.0001</max_step_size>
    <real_time_factor>1</real_time_factor>
    <real_time_update_rate>10000</real_time_update_rate>

The sampling rate of the IMU was set to 500 Hz in `corin/corin_description/robots/Corin_model.urdf.xacro`.

To run the estimator simulation:
1) roslaunch corin_gazebo corin_world.launch
2) roslaunch corin_control position_control.launch
3) rosrun corin_control main.py
4) rosparam set /walkforward true
5) rosrun corin_control State_Estimator_test_new.py
6) rosrun rqt_plot rqt_plot
	- /body_r/vector/x (ground truth)
	- /body_r/vector/y
	- /body_r/vector/z
	- /imu_r/x (estimated state)
	- /imu_r/y
	- /imu_r/	
7) Execute Path? (Accept-y Reject-n) : y


The file `corin/corin_control/py_script/State_Estimator_test_new.py` runs the state estimator in simulation mode. The `self.T_sim` and `self.T_imu` must match the sampling rate of the simulation and IMU as described above. The tuning parameter are desribed in the journal paper. 

Set the `self.data_source` to “imu” to use the Gazebo IMU data (which is very inaccurate), and set it to “imu2” which uses the Gazebo model position and orientation in order to derive the ideal accelerometer and gyroscope values.

`corin/corin_control/py_script/library/state_estimator_new.py` is the implementation of the EKF as a state estimator for Corin. This file has been integrated in the `closed_loop` branch on Bitbucket.

An older implementation of the state estimator using only kinematics, based on the “A Leg Configuration Measurement System for Full-Body Pose Estimates in a Hexapod Robot” by Pei-Chun Lin, is also available.

### IMU Characterisation

In order to estimate the white noise covariance and bias term covariance of the IMU the following was performed. The MATLAB files are from the https://github.com/rpng/kalibr_allan project.
1) The IMU was left on a stable surface, ensuring that there was no disturbance.
2) Use `corin/corin_control/py_script/record_IMU.sh` to log the IMU values for a few hours.
3) Use `corin/corin_control/py_script/extract_IMU_bag.sh` to convert the logged data into a csv file.
4) Move the csv file to `Dropbox/Corin/Matlab/IMU_Allen_Variance_MATLAB`.
5) Use `Dropbox/Corin/Matlab/IMU_Allen_Variance_MATLAB/SCRIPT_allan_matparallel.m` to evaluate the Allen Variance values. (use the csv file name in the script)
6) Use `Dropbox/Corin/Matlab/IMU_Allen_Variance_MATLAB/SCRIPT_process_results.m` to display the results. (use the results file name in the script)

## State Estimator Experiments

### VICON Tracking

To generate ground truth data, a VICON system was used to track Corin. Initially, the VICON system was calibrated. The VICON PC was connected to a wifi access point (router). The IP address of the VICON PC was noted. 

Using the https://github.com/ethz-asl/vicon_bridge VICON ROS package on the PC running the robot controller, the VICON measurements were streamed in. The IP address was correctly set in the `vicon.launch` file.

Tracking was performed by attaching reflective spheres, provided by VICON, on Corin. These were screwed to a laser-cut attachment on Corin as shown in the figure below. This allowed the VICON object frame to be aligned with the robot body frame. In the VICON Tracker software, the origin of the VICON object frame was placed at sphere 2. The x axis was aligned with sphere 3, and the y axis was aligned with sphere 1. This is shown in the figure. The method of transforming the VICON object position and orientation measurements to the correct frame is described in the journal paper. 


FIGURE VICON spheres


### Moving Corin

The following commands were used to move Corin a certain distance baesd on different motion parameters.

1) roscore
2) roslaunch corin_manager corin_manager.launch
3) rosrun corin_control main.py
4) rosparam set /walkforward true OR rosparam set /walkright true OR  rosparam set /rotate true

The parameters changed include GAIT_TYPE and STEP_STROKE defined in `corin/corin_control/py_script/library/constants.py`.

The distance travelled was modified in `corin/corin_control/py_script/control_interface.py`.

As for wall walking, the reactive_controller branch was used.

### Running the IMU

The LORD IMU was run using th

In the `microstrain_3dm.cpp` file, the follwing lines (in two separate locations) were changed from 

    // put into ENU - swap X/Y, invert Z
    nav_msg_.pose.pose.orientation.x = curr_filter_quaternion_.q[2];
    nav_msg_.pose.pose.orientation.y = curr_filter_quaternion_.q[1];
    nav_msg_.pose.pose.orientation.z = -1.0*curr_filter_quaternion_.q[3];
    nav_msg_.pose.pose.orientation.w = curr_filter_quaternion_.q[0];

to 

    nav_msg_.pose.pose.orientation.x = curr_filter_quaternion_.q[1];
    nav_msg_.pose.pose.orientation.y = curr_filter_quaternion_.q[2];
    nav_msg_.pose.pose.orientation.z = curr_filter_quaternion_.q[3];
    nav_msg_.pose.pose.orientation.w = curr_filter_quaternion_.q[0];

The port value in the `microstrain_25.launch` file was changed to match that of the connected device. The IMU was attached to the robot in the position show in the figure below.

FIG IMU

### Force Sensors

For the state estimation experiments, the old force sensor driver (on Bitcuket) was used. The Corin launch file was `variense_driver/FSE103.py`. The published topics were of the form `force_vector_#`. Once the sensors are up and running, they have to be reset using the following command, while the sensors were not measuring any load (feet suspended in air):

    rosservice call force_sensor_0/reset_offset

and similarly for the other 5 sensors.

However, the new sensor driver can be found at https://github.com/h-khalili/fse103_ros_driver.

### Logging Data

The `corin/corin_control/py_script/record_bag_hardware.sh` was used to record the experiment data to a rosbag file. 

The rosbag file name for each experiment was of the form:

    corin_<motion>_<gait>_<stepsize>.bag

The motions were:
- carpet_front: walking forward on the green carpet in F46
- front: walking forward
- side: walking sideways
- rotate: rotating
- wall: wall walking.

The files were placed in a separate ROS package, to avoid tracking these on BitBucket, in the following directory:
catkin_ws/src/data/experiments_2018_Dec


### Displaying Experimental Data

The file `corin/corin_control/py_script/experiment_numbers.py` creates an array of file paths to the experiment data. The experiment number is the index of the array referring to a specific experiment.

`corin/corin_control/py_script/Corin_VICON_tracking.py` allows the plotting of the VICON trajectories recorded during the experiments. The file is called in the command line followed by the desired experiment number as follows:

    rosrun corin_control Corin_VICON_tracking.py 29

Similarly, the `corin/corin_control/py_script/Corin_trajectories_transformed.py` can be used to plot the trajectory of the robot body in the world frame, including that of foot 2 (third foot on the left).

`corin/corin_control/py_script/Corin_force_sensor_read.py` can be used to read the force sensor measurements for a specified experiment.

### Offline State Estimation

The `corin/corin_control/py_script/Corin_State_Estimator_offline_test_new.py` applies the EKF state estimator to the data collected during a specified experimental run. For example, to see the results for experiment 29 run the following command:

    rosrun corin_control Corin_State_Estimator_offline_test_new.py 29

This uses the implementation of the state estimator in the `state_estimator_new.py` file. 

For an alternative implementation of covariance initialisation see the `corin/corin_control/py_script/Corin_State_Estimator_offline_test_P0.py` file. This implements the initilisation described in the journal paper. However, it does not seem to produce better results. This method can be ignored.


## Experimental Data Analysis

### Exhaustive Search

`corin/corin_control/py_script/Corin_State_Estimator_optimisation.py` simply goes through all the combinations of input parameters to the state estimator, as specified in the file, and applies the algorithm to the measurements recorded during the specified experiment. The state estimtor error results were logged and stored in the `/catkin_ws/src/data/2019_optimisation_results` folder.

`corin/corin_control/py_script/EKF_min_error.py` lists the best performing parameter combinations for a given experiment.

`corin/corin_control/py_script/EKF_optimum.py` prints out the parameter combinations with the least error across a list of experiments.

However, this method did not have satisfactory results.

### Sensitivity Analysis

`corin/corin_control/py_script/Corin_State_Estimator_sensitivity_3.py` and `Corin_State_Estimator_sensitivity_6.py` scripts were used to investigate the sensitivity of the state estimator performance to each of the tuning parameters. The former included state estimator initialisation while the latter did not.

Essentially all the parameters are kept at their default value, while the others were changed one at a time. The results were stored in the `catkin_ws/src/data/2019_sensitivity_results_3` and `2019_sensitivity_results_6` folders. 

`Corin_State_Estimator_sensitivity_analysis_3.py` and `Corin_State_Estimator_sensitivity_analysis_6.py` and `Corin_State_Estimator_sensitivity_analysis_3n6.py` can be used to plot the results of the analysis.

The `corin/corin_control/py_script/Corin_State_Estimator_sensitivity_combine_3.py` and `Corin_State_Estimator_sensitivity_combine_6.py` scripts were used to combine all the results from the different experiments into single files for each of the varied parameters. The resulting files were stored in the same folder. For example, `2019_sensitivity_results_Qf_position_error_3.csv` contains all the position error results from the different experiments for the accelerometer covariance sweeps.

These files were used to plot the sensitivity sweep figures used in the journal RAS journal paper.

Finally, the `Corin_State_Estimator_sensitivity_comparison_3.py` script was used to generate the data to plot the sensitivity bar plot in the journal. The results it generated were saved in `catkin_ws/src/data/2019_sensitivity_comparison_3.csv`.

### Particle Swarm Optimisation

In order to implement PSO to optimise the state estimator for one experiment, use `corin/corin_control/py_script/Corin_State_Estimator_PSO_optimisation`. 

However, to optimise across different experiments either use 
- `Corin_State_Estimator_PSOs_optimisation.py`
- `Corin_State_Estimator_PSOs_optimisation_array_max.py`

The following parameters can be changed for optimisation:
- The exp_list selects which experiments to optimise for. 
- The pool parameter determines how many CPU cores to use. 10 seems to be the best number. 12 throttles the CPU.

In the former file, change 

    new_value = np.sqrt(np.mean(np.square(results)))

to

    new_value = max(results)

to optimise for the maximum error across all experiments vs their rms error.

In the latter file, the optimisation is firstly done for the maximum error to the nearest 0.1%. If that is satisfied, the second-highest error is optimised for, etc.

The results of the optimisation are stored in `catkin_ws/src/data/PSO`.

## Impedance Control

`corin/corin_control/py_script/robot_impedence.py` implements an impedence controller on each leg, maintaining a desired force value on each foot.

`corin/corin_control/py_script/robot_closed_loop.py` implements an impedence controller on each leg, while the robot walks forward. The added code is in the trajectory_tracking function.

