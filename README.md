# Corin - Demo Branch
Branch for simple demonstrations of Corin. Based on the simple_demo branch, but with some changes. If in doubt, this is the one that should be used.

The original author of this code was Dr Wei Cheah. It is now being maintained by Daniel S. Johnson on behalf of the Centre for Robotics and AI at the University of Manchester. I can be contacted at: [daniel.johnson-5@student.manchester.ac.uk](daniel.johnson-5@student.manchester.ac.uk).

## Usage

### Starting the Demo

To run the standard demo requires two terminals.

In the first terminal, launch the corin manager.

``` bash
source cakin_ws/devel/setup.bash
roslaunch corin_manager corin_manager.launch
```

Due to issues with the USB drivers on the Corin workstation, when working on that machine the corin manager must be launched from a terminal as root. Therefore, users should first enter `sudo bash` in order to obtain root permissions. Any Conda environments which may be activated by default should also be deactivated (`conda deactivate`).

Wait for a few seconds after launching, the robot should move to its stand up position.

In the second terminal, launch the robot's controller:

```bash
source cakin_ws/devel/setup.bash
roslaunch corin_control main.py
```

This should begin the controller loop. The loop currently consists of the robot moving around in a 1 meter square and occasionally rotating the torso. The loop requires minimal human interaction, but you will need to keep an eye on the umbilical cable to make sure the robot doesn't unplug itself or slip on the cable. The position control is also open loop, so the robot will gradually drift away from where the controller thinks it is. Thus, occasional resets of the manager may be required.

### Controlling the Robot

The controller waits for a message telling it what to do, which can came from setting rosparams to true or from the control loop itself. This is currently set up to loop through several actions, but manual control can be achieved by commenting out the looping sections of `corin_control/py_script/main.py`. This is also how the loop sequence can be edited.

Previously, the manager was configured to confirm with the user explicitly before moving, to catch bugs or params set high that were not intended. This has been disabled to allow the loop to execute with minimum human interaction, but it should be re-enabled if manual control is being used. This can be done by adjusting the commented sections on lines 489 and 490 of `corin_control/py_script/robot_controller.py`.

In manual control, the actions can be triggered by using a third terminal to set the relevant rosparam to `true`. The controller will notice this, execute the motion, and then reset the param to `false`. The relevant params are:

- /bodypose: The robot moves the torso around while keeping the feet still, intended to demonstrate its dexterity. **Do not activate this one while the robot is on its pedestal as it will collide with the table!**
- /rotate: Turn 90 degrees (or so) to the left
- /walkback: Walk 1 metre backwards
- /walkforward: Walk 1 metre forwards
- /walkleft: Walk 1 metre left
- /walkright: Walk 1 metre right

Note: The robot keeps track of which way it thinks it's facing and interprets the directions (forward, backward, left, right) as relative to the direction it started facing when the manager was launched. Thus, if the robot turns 90 degrees to the left, and then executes `walkforward`, then it will walk in the direction which is now its right.

### Stopping the Demos

When the manager node is killed (via CTRL+C) the robot will immediately stop moving. The motors will continue to hold their positions and draw power, to prevent the robot from collapsing. If the manager is then restarted, then the robot will curl up and assume it's resting pose in a controlled manner. The manager can then be interrupted again once the robot is in a safe position. At this point the power to the robot can be safely cut.

### Editing the Demos

The settings for the demo are effective as they are currently, and changing things is discouraged as the original author of the code is not available to provide assistance if things go wrong. However, if one really must make changes, most of the robot's parameters (including the gait used) can be found in `corin_control/py_script/library/constant.py`
