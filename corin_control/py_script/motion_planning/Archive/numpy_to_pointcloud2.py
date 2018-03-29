#!/usr/bin/env python

import sys; sys.dont_write_bytecode = True

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField

import numpy as np

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
    points_arr['z'] = data[2]
    points_arr['r'] = 0
    points_arr['g'] = 0
    points_arr['b'] = 255
    
    return points_arr

def makeArray(npoints):
    """ create random 3D array of size npoints """

    points_arr = np.zeros((npoints,), dtype=[
                        ('x', np.float32),
                        ('y', np.float32),
                        ('z', np.float32),
                        ('r', np.uint8),
                        ('g', np.uint8),
                        ('b', np.uint8)])
    points_arr['x'] = np.arange(0,1.0,0.1)#np.random.random((npoints,))
    points_arr['y'] = np.arange(0,1.0,0.1)#np.random.random((npoints,))
    points_arr['z'] = np.arange(0,1.0,0.1)#np.random.random((npoints,))
    points_arr['r'] = 0#np.floor(np.random.random((npoints,))*255)
    points_arr['g'] = 0
    points_arr['b'] = 255
    x = np.random.random((npoints,))

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
