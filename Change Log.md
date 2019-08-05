# List of temporary/hack modifications

1) In path_planner.py, added line 834-837 for setting foot position in world frame
2) Disabled leg boundary from suspending robot motion
3) Robot stays in place for stamping test.

# Permanent Changes
In corin_control:
1) Leg class:
	- Removed Kim2007 passivity on reducing impact force.
	- Contact forces updated using actual joint position.
	- world_X_foot position updated using actual joint position.
	- Function to reset all admittance controller.
2) Robot class:
	- Added base controller - PID class type.
	- Leg admittance controller calculated here, offset not sum (+=) to XHd.coxa_X_foot.
3) Admittance controller class:
	- Added reset function.
2) Added PID controller class
a) RViZ visual:
	- Added cone visualization.
	- Updated force vectors.
a) Added rviz_visual package into Corin metapackage
3) Robot controller:
	- Leg updated to original position instead of gravity compensation prior to generating transfer phase spline
	- Leg admittance controllers reset added after leg becomes transfer phase
	- State machine modified for gait phases to updated only when required
