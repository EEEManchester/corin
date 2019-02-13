#!/usr/bin/env python

""" Functions to convert numpy arrays to ROS messages
	1) Converts a TrajectoryPoints array to nav_msgs/Path:
		array_to_path(path, stamp, frame_id)
	2) Converts a numpy array to nav_msgs/Path:
		array_to_marker_array(arrays, stamp, frame_id)
	3) Converts one-dimensional array to JointState msg:
		array_to_joint_states(arrays, stamp, frame_id)
"""

import sys; sys.dont_write_bytecode = True

import numpy as np
import itertools
from traits import *
import rospy
from std_msgs.msg import Header
## Path
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
## Marker
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
## PCL
from sensor_msgs.msg import PointCloud2, PointField
## JointStates
from sensor_msgs.msg import JointState
## Pose
from geometry_msgs.msg import Pose
## PlanPath
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import MultiDOFJointTrajectoryPoint
from nav_msgs.msg import Path
from std_msgs.msg import *
## ======================================================================================================================================== ##
## 																	Pose																	##
## ======================================================================================================================================== ##
def array_to_pose(data):
	""" converts numpy Re^(6) to geometry_msgs/Pose, euler angles used """

	pose = Pose()
	pose.position.x = data.item(0)
	pose.position.y = data.item(1)
	pose.position.z = data.item(2)
	pose.orientation.x = data.item(3)
	pose.orientation.y = data.item(4)
	pose.orientation.z = data.item(5)

	return pose
	
## ======================================================================================================================================== ##
## 																	Path																	##
## ======================================================================================================================================== ##

def array_to_path(path_arr, stamp=None, frame_id=None, offset=None):
	""" Converts a TrajectoryPoints array to nav_msgs/Path """
	## TODO: CHANGE TO 2D ARRAY?

	## Define Variables ##
	path_msg = Path()

	if stamp is not None:
		path_msg.header.stamp = stamp
	if frame_id is not None:
		path_msg.header.frame_id = frame_id
	if offset is None:
		offset = [0]*6
	
	try:
		for i in range (0,len(path_arr.X.t)):
			pose = PoseStamped()
			pose.header.stamp 	 = rospy.Time(path_arr.X.t[i])
			pose.header.frame_id = frame_id
			pose.pose.position.x = path_arr.X.xp[i][0] + offset[0]
			pose.pose.position.y = path_arr.X.xp[i][1] + offset[1]
			pose.pose.position.z = path_arr.X.xp[i][2] + offset[2]
			
			quat = tf.transformations.quaternion_from_euler(path_arr.W.xp[i][0] + offset[3], 
															path_arr.W.xp[i][1] + offset[4], 
															path_arr.W.xp[i][2] + offset[5]) 
			
			pose.pose.orientation.x = quat[0]
			pose.pose.orientation.y = quat[1]
			pose.pose.orientation.z = quat[2]
			pose.pose.orientation.w = quat[3]

			path_msg.poses.append(pose)
	except:
		for i in range (0, len(path_arr)):
			pose = PoseStamped()
			pose.header.stamp 	 = rospy.Time(i)
			pose.header.frame_id = frame_id
			pose.pose.position.x = path_arr[i][0] + offset[0]
			pose.pose.position.y = path_arr[i][1] + offset[1]
			pose.pose.position.z = path_arr[i][2] + offset[2]
			
			quat = tf.transformations.quaternion_from_euler(path_arr[i][0] + offset[3], 
															path_arr[i][1] + offset[4], 
															path_arr[i][2] + offset[5]) 
			
			pose.pose.orientation.x = quat[0]
			pose.pose.orientation.y = quat[1]
			pose.pose.orientation.z = quat[2]
			pose.pose.orientation.w = quat[3]

			path_msg.poses.append(pose)

	return path_msg

## ======================================================================================================================================== ##
## 																Marker Array																##
## ======================================================================================================================================== ##

def array_to_marker_array(path_arr, stamp=None, frame_id=None):
	""" Converts a numpy array to nav_msgs/Path """

	## Define Variables ##
	mark_array = MarkerArray()

	## assumes that footholds array are of the same size
	for i in range (0,len(path_arr[0])):
		for j in range(0,6):
			mark = Marker()

			if stamp is not None:
				mark.header.stamp = stamp
			if frame_id is not None:
				mark.header.frame_id = frame_id

			mark.type = 2
			mark.id = 6*i + j
			mark.pose.position.x = path_arr[j][i][0]
			mark.pose.position.y = path_arr[j][i][1]
			mark.pose.position.z = 0.03
			mark.pose.orientation.x = 0.0;
			mark.pose.orientation.y = 0.0;
			mark.pose.orientation.z = 0.0;
			mark.pose.orientation.w = 1.0;
			mark.scale.x = 0.03
			mark.scale.y = 0.03
			mark.scale.z = 0.03
			mark.color.a = 1.0; # transparency level
			# colour scheme for each foot
			if (j == 0):
				mark.color.r = 1.0;
				mark.color.g = 0.0;
				mark.color.b = 0.0;
			elif (j == 1):
				mark.color.r = 0.0;
				mark.color.g = 1.0;
				mark.color.b = 0.0;
			elif (j == 2):
				mark.color.r = 0.0;
				mark.color.g = 0.0;
				mark.color.b = 1.0;
			elif (j == 3):
				mark.color.r = 1.0;
				mark.color.g = 0.27;
				mark.color.b = 0.0;
			elif (j == 4):
				mark.color.r = 0.0;
				mark.color.g = 0.98;
				mark.color.b = 0.6;
			elif (j == 5):
				mark.color.r = 0.75;
				mark.color.g = 0.24;
				mark.color.b = 1.0;
				
			# stack into array
			mark_array.markers.append(mark)

	return mark_array

def foothold_list_to_marker_array(path_arr, stamp=None, frame_id=None):
	""" Converts custom marker list to nav_msgs/Path """

	## Define Variables ##
	mark_array = MarkerArray()
	
	## assumes that footholds array are of the same size
	for j in range(0,6):
		for i in range (0,len(path_arr[j].t)):
			mark = Marker()

			if stamp is not None:
				mark.header.stamp = stamp
			if frame_id is not None:
				mark.header.frame_id = frame_id
			
			mark.type = 2
			mark.id = 6*i + j
			mark.pose.position.x = path_arr[j].xp[i][0]
			mark.pose.position.y = path_arr[j].xp[i][1]
			mark.pose.position.z = path_arr[j].xp[i][2]
			mark.pose.orientation.x = 0.0;
			mark.pose.orientation.y = 0.0;
			mark.pose.orientation.z = 0.0;
			mark.pose.orientation.w = 1.0;
			mark.scale.x = 0.03
			mark.scale.y = 0.03
			mark.scale.z = 0.03
			mark.color.a = 1.0; # transparency level
			# colour scheme for each foot
			if (j == 0):
				mark.color.r = 1.0;
				mark.color.g = 0.0;
				mark.color.b = 0.0;
			elif (j == 1):
				mark.color.r = 0.0;
				mark.color.g = 1.0;
				mark.color.b = 0.0;
			elif (j == 2):
				mark.color.r = 0.0;
				mark.color.g = 0.0;
				mark.color.b = 1.0;
			elif (j == 3):
				mark.color.r = 1.0;
				mark.color.g = 0.27;
				mark.color.b = 0.0;
			elif (j == 4):
				mark.color.r = 0.0;
				mark.color.g = 0.98;
				mark.color.b = 0.6;
			elif (j == 5):
				mark.color.r = 0.75;
				mark.color.g = 0.24;
				mark.color.b = 1.0;
				
			# stack into array
			mark_array.markers.append(mark)

	return mark_array

def list_to_marker_array(path_arr, stamp=None, frame_id=None):
	""" Converts custom marker list to nav_msgs/Path """

	## Define Variables ##
	mark_array = MarkerArray()

	## assumes that footholds array are of the same size
	
	for i in range (0,len(path_arr)):
		xb = path_arr[i].flatten()
		mark = Marker()

		if stamp is not None:
			mark.header.stamp = stamp
		if frame_id is not None:
			mark.header.frame_id = frame_id

		mark.type = 2
		mark.id = i 
		mark.pose.position.x = xb[0]
		mark.pose.position.y = xb[1]
		mark.pose.position.z = xb[2]
		mark.pose.orientation.x = 0.0;
		mark.pose.orientation.y = 0.0;
		mark.pose.orientation.z = 0.0;
		mark.pose.orientation.w = 1.0;
		mark.scale.x = 0.015
		mark.scale.y = 0.015
		mark.scale.z = 0.015
		mark.color.a = 1.0; # transparency level
		
		# colour scheme
		mark.color.r = 1.0;
		mark.color.g = 0.0;
		mark.color.b = 0.0;
					
		# stack into array
		mark_array.markers.append(mark)

	return mark_array
## ======================================================================================================================================== ##
## 																Joint States																##
## ======================================================================================================================================== ##
# Robot specific joint Names 
# TODO: move elsewhere
JOINT_NAME = {}
JOINT_NAME[0]  = 'lf_q1_joint';	JOINT_NAME[3]  = 'lm_q1_joint'; JOINT_NAME[6]  = 'lr_q1_joint';
JOINT_NAME[1]  = 'lf_q2_joint';	JOINT_NAME[4]  = 'lm_q2_joint'; JOINT_NAME[7]  = 'lr_q2_joint';
JOINT_NAME[2]  = 'lf_q3_joint';	JOINT_NAME[5]  = 'lm_q3_joint'; JOINT_NAME[8]  = 'lr_q3_joint';

JOINT_NAME[9]  = 'rf_q1_joint';	JOINT_NAME[12] = 'rm_q1_joint';	JOINT_NAME[15] = 'rr_q1_joint';
JOINT_NAME[10] = 'rf_q2_joint';	JOINT_NAME[13] = 'rm_q2_joint';	JOINT_NAME[16] = 'rr_q2_joint';
JOINT_NAME[11] = 'rf_q3_joint';	JOINT_NAME[14] = 'rm_q3_joint';	JOINT_NAME[17] = 'rr_q3_joint';

def array_to_joint_states(q_arr, stamp=None, frame_id=None):
	""" Converts one-dimensional array to JointState msg """

	## Define Variables ##
	q = JointState()

	if stamp is not None:
		q.header.stamp = stamp
	if frame_id is not None:
		q.header.frame_id = ''

	## size should be 18
	for i in range (0,len(q_arr)):
		q.name.append(JOINT_NAME[i])
		q.position.append(q_arr[i])
	
	return q

## ======================================================================================================================================== ##
## 																Point Cloud																	##
## ======================================================================================================================================== ##

# mappings between PointField types and numpy types
type_mappings = [(PointField.INT8, np.dtype('int8')), (PointField.UINT8, np.dtype('uint8')), (PointField.INT16, np.dtype('int16')),
                 (PointField.UINT16, np.dtype('uint16')), (PointField.INT32, np.dtype('int32')), (PointField.UINT32, np.dtype('uint32')),
                 (PointField.FLOAT32, np.dtype('float32')), (PointField.FLOAT64, np.dtype('float64'))]
pftype_to_nptype = dict(type_mappings)
nptype_to_pftype = dict((nptype, pftype) for pftype, nptype in type_mappings)

def point_cloud_array_mapping(data):
    """ remaps from 3D array to required output form """

    npoints = len(data[0])
    points_arr = np.zeros((npoints,), dtype=[
                        ('x', np.float32),
                        ('y', np.float32),
                        ('z', np.float32),
                        ('r', np.uint8),
                        ('g', np.uint8),
                        ('b', np.uint8)])
    
    points_arr['x'] = data[0]
    points_arr['y'] = data[1]
    points_arr['z'] = data[2] - 0.015
    points_arr['r'] = 0
    points_arr['g'] = 0
    points_arr['b'] = 255
    
    return points_arr

def dtype_to_fields(dtype):
    '''Convert a numpy record datatype into a list of PointFields.
    '''
    fields = []
    for field_name in dtype.names:
        np_field_type, field_offset = dtype.fields[field_name]
        pf = PointField()
        pf.name = field_name
        if np_field_type.subdtype:
            item_dtype, shape = np_field_type.subdtype
            pf.count = np.prod(shape)
            np_field_type = item_dtype
        else:
            pf.count = 1

        pf.datatype = nptype_to_pftype[np_field_type]
        pf.offset = field_offset
        fields.append(pf)
    return fields

def array_to_pointcloud2(cloud_arr, stamp=None, frame_id=None):
    '''Converts a numpy record array to a sensor_msgs.msg.PointCloud2.
    '''
    # make it 2d (even if height will be 1)
    ## Define Variables ##
    cloud_arr = np.atleast_2d(cloud_arr)

    cloud_msg = PointCloud2()

    if stamp is not None:
        cloud_msg.header.stamp = stamp
    if frame_id is not None:
        cloud_msg.header.frame_id = frame_id
    
    cloud_msg.height = cloud_arr.shape[0]
    cloud_msg.width = cloud_arr.shape[1]
    cloud_msg.fields = dtype_to_fields(cloud_arr.dtype)
    cloud_msg.is_bigendian = False # assumption
    cloud_msg.point_step = cloud_arr.dtype.itemsize
    cloud_msg.row_step = cloud_msg.point_step*cloud_arr.shape[1]
    cloud_msg.is_dense = all([np.isfinite(cloud_arr[fname]).all() for fname in cloud_arr.dtype.names])
    cloud_msg.data = cloud_arr.tostring()

    return cloud_msg

## ======================================================================================================================================== ##
## 																Graph and MultiArray Conversion																	##
## ======================================================================================================================================== ##
def graph_attr_to_multiarray(GridMap, attr):
	""" Converts NetworkX grid map .5 data into ROS multiarray format """
	
	layer  = Float32MultiArray()
	layout = MultiArrayLayout()
	dim = MultiArrayDimension()

	layer.data, depth = GridMap.graph_attributes_to_nparray(attr)

	dim.label = attr
	dim.size = GridMap.map_size_g[1]
	dim.stride = depth*GridMap.map_size_g[0]*GridMap.map_size_g[1]
	layout.dim.append(dim)
	dim = MultiArrayDimension()
	dim.label = "width"
	dim.size = GridMap.map_size_g[0]
	dim.stride = depth*GridMap.map_size_g[0]
	layout.dim.append(dim)
	dim = MultiArrayDimension()
	dim.label = "length"
	dim.size = depth
	dim.stride = depth
	layout.dim.append(dim)
	layer.layout = layout

	return layer

def multiarray_to_graph_attr(GridMap, attr):
	""" Converts ROS multiarray format to NetworkX grid map .5 data  """

	pass
## ======================================================================================================================================== ##
## 																PlanPath and MotionPlan Conversion																	##
## ======================================================================================================================================== ##

def motionplan_to_planpath(motion_plan, frame_id=None):
	""" Converts motion plan to PlanPath service message """

	qbp = MultiDOFJointTrajectoryPoint()
	qbi = Path()
	gphase = Int8MultiArray()
	wXf = Float64MultiArray()
	bXf = Float64MultiArray()
	bXN = Float64MultiArray()
	
	for i in range(0, len(motion_plan.qb.X.t)):
		
		gtf = Transform()
		vtf = Twist()
		atf = Twist()

		gtf.translation.x = motion_plan.qb.X.xp[i][0]
		gtf.translation.y = motion_plan.qb.X.xp[i][1]
		gtf.translation.z = motion_plan.qb.X.xp[i][2]
		gtf.rotation.x = motion_plan.qb.W.xp[i][0]
		gtf.rotation.y = motion_plan.qb.W.xp[i][1]
		gtf.rotation.z = motion_plan.qb.W.xp[i][2]

		vtf.linear.x = motion_plan.qb.X.xv[i][0]
		vtf.linear.y = motion_plan.qb.X.xv[i][1]
		vtf.linear.z = motion_plan.qb.X.xv[i][2]
		vtf.angular.x = motion_plan.qb.W.xv[i][0]
		vtf.angular.y = motion_plan.qb.W.xv[i][1]
		vtf.angular.z = motion_plan.qb.W.xv[i][2]
	
		atf.linear.x = motion_plan.qb.X.xa[i][0]
		atf.linear.y = motion_plan.qb.X.xa[i][1]
		atf.linear.z = motion_plan.qb.X.xa[i][2]
		atf.angular.x = motion_plan.qb.W.xa[i][0]
		atf.angular.y = motion_plan.qb.W.xa[i][1]
		atf.angular.z = motion_plan.qb.W.xa[i][2]

		qbp.transforms.append(gtf)
		qbp.velocities.append(vtf)
		qbp.accelerations.append(atf)
	qbp.time_from_start = rospy.Time(motion_plan.qb.X.t[1] - motion_plan.qb.X.t[0])
	
	for i in range(0, len(motion_plan.qbp)):
		ip = PoseStamped()

		if frame_id is not None:
			ip.header.frame_id = frame_id
		ip.header.stamp = rospy.Time(motion_plan.qb.X.t[i])

		ip.pose.position.x = motion_plan.qbp[i][0]
		ip.pose.position.y = motion_plan.qbp[i][1]
		ip.pose.position.z = motion_plan.qbp[i][2]
		ip.pose.orientation.x = motion_plan.qbp[i][3]
		ip.pose.orientation.y = motion_plan.qbp[i][4]
		ip.pose.orientation.z = motion_plan.qbp[i][5]
		ip.pose.orientation.w = 0

		qbi.poses.append(ip)
	
	dheader, ddata = list_to_multiarray(motion_plan.gait_phase)
	gphase.layout.dim.append(dheader)
	gphase.data = ddata
	
	for j in range(0,6):
		dheader, ddata = list_to_multiarray(motion_plan.f_world_X_foot[j].xp)
		wXf.layout.dim.append(dheader)
		wXf.data += ddata

		dheader, ddata = list_to_multiarray(motion_plan.f_base_X_foot[j].xp)
		bXf.layout.dim.append(dheader)
		bXf.data += ddata

		dheader, ddata = list_to_multiarray(motion_plan.f_world_base_X_NRP[j].xp)
		bXN.layout.dim.append(dheader)
		bXN.data += ddata

	return qbp, qbi, gphase, wXf, bXf, bXN

def list_to_multiarray(idata):

	adim = MultiArrayDimension()
	adim.size = len(idata[0])
	adim.stride = len(idata)
	
	return adim, list(itertools.chain.from_iterable(idata))

def planpath_to_motionplan(plan_path):
	""" Converts PlanPath service message to MotionPlan """
	
	motion_plan = MotionPlan()

	## Trajectory parameters
	tint = plan_path.base_path.time_from_start.to_sec()
	path_size = len(plan_path.base_path.transforms)

	motion_plan.qb.X.t = [None]*path_size
	motion_plan.qb.W.t = [None]*path_size
	motion_plan.qb.X.xp = np.zeros((path_size,3))
	motion_plan.qb.X.xv = np.zeros((path_size,3))
	motion_plan.qb.X.xa = np.zeros((path_size,3))
	motion_plan.qb.W.xp = np.zeros((path_size,3))
	motion_plan.qb.W.xv = np.zeros((path_size,3))
	motion_plan.qb.W.xa = np.zeros((path_size,3))

	for i in range(0, path_size):
		motion_plan.qb.X.xp[i,0:3] = np.array([ plan_path.base_path.transforms[i].translation.x,
													 									plan_path.base_path.transforms[i].translation.y,
													 									plan_path.base_path.transforms[i].translation.z ])

		motion_plan.qb.X.xv[i,0:3] = np.array([ plan_path.base_path.velocities[i].linear.x,
													 									plan_path.base_path.velocities[i].linear.y,
													 									plan_path.base_path.velocities[i].linear.z ])

		motion_plan.qb.X.xa[i,0:3] = np.array([ plan_path.base_path.accelerations[i].linear.x,
													 									plan_path.base_path.accelerations[i].linear.y,
													 									plan_path.base_path.accelerations[i].linear.z ])

		motion_plan.qb.W.xp[i,0:3] = np.array([ plan_path.base_path.transforms[i].rotation.x,
													 									plan_path.base_path.transforms[i].rotation.y,
													 									plan_path.base_path.transforms[i].rotation.z ])

		motion_plan.qb.W.xv[i,0:3] = np.array([ plan_path.base_path.velocities[i].angular.x,
													 									plan_path.base_path.velocities[i].angular.y,
													 									plan_path.base_path.velocities[i].angular.z ])

		motion_plan.qb.W.xa[i,0:3] = np.array([ plan_path.base_path.accelerations[i].angular.x,
													 									plan_path.base_path.accelerations[i].angular.y,
													 									plan_path.base_path.accelerations[i].angular.z ])

		motion_plan.qb.X.t[i] = i*tint
		motion_plan.qb.W.t[i] = i*tint

	for i in range(0, len(plan_path.CoB.poses)):
		motion_plan.qbp.append([ plan_path.CoB.poses[i].pose.position.x,
							 							 plan_path.CoB.poses[i].pose.position.y,
							 							 plan_path.CoB.poses[i].pose.position.z,
							 							 plan_path.CoB.poses[i].pose.orientation.x,
							 							 plan_path.CoB.poses[i].pose.orientation.y,
							 							 plan_path.CoB.poses[i].pose.orientation.z ])
	## Remap Gait Phase List
	for i in range(0, int(len(plan_path.gait_phase.data)/6)):
		motion_plan.gait_phase.append(list(plan_path.gait_phase.data[i*6:i*6+6]))
	
	## Remap Foot Contact List
	nc_wXf = nc_bXf = nc_wbXN = 0
	for j in range(0,6):
		
		for i in range(0, plan_path.f_world_X_foot.layout.dim[j].stride):
			motion_plan.f_world_X_foot[j].xp.append(np.array(plan_path.f_world_X_foot.data[nc_wXf*3+i*3:nc_wXf*3+i*3+3]).reshape((3,1)))
			motion_plan.f_world_X_foot[j].t.append(i)
		nc_wXf += plan_path.f_world_X_foot.layout.dim[j].stride

		for i in range(0, plan_path.f_base_X_foot.layout.dim[j].stride):
			motion_plan.f_base_X_foot[j].xp.append(np.array(plan_path.f_base_X_foot.data[nc_bXf*3+i*3:nc_bXf*3+i*3+3]).reshape((3,1)))
			motion_plan.f_base_X_foot[j].t.append(i)
		nc_bXf += plan_path.f_base_X_foot.layout.dim[j].stride

		for i in range(0, plan_path.f_world_base_X_NRP.layout.dim[j].stride):
			motion_plan.f_world_base_X_NRP[j].xp.append(np.array(plan_path.f_world_base_X_NRP.data[nc_wbXN*3+i*3:nc_wbXN*3+i*3+3]).reshape((3,1)))
			motion_plan.f_world_base_X_NRP[j].t.append(i)
		nc_wbXN += plan_path.f_world_base_X_NRP.layout.dim[j].stride

	return motion_plan
