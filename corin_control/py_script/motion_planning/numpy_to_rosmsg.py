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
		print 'path length: ', len(path_arr.X.t)
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
    # print cloud_arr
    # print 'max x', np.amax(cloud_arr.shape[0])
    # print 'max y', np.amax(cloud_arr.shape[1])
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
## 																Something																	##
## ======================================================================================================================================== ##