#!/usr/bin/env python

import numpy as np
import math

""" Transformation and matrix operation for arrays 
	Some functions adopted from Christoph Gohlke
"""
__version__ = '1.0'
__author__ = 'Wei Cheah'

######################################################################
##                          Vector Operations                       ##
######################################################################

def unit_vector(data, axis=None, out=None):
    """Return ndarray normalized by length, i.e. Euclidean norm, along axis. """

    if out is None:
        data = np.array(data, dtype=np.float64, copy=True)
        if data.ndim == 1:
            data /= math.sqrt(np.dot(data, data))
            return data
    else:
        if out is not data:
            out[:] = np.array(data, copy=False)
        data = out
    length = np.atleast_1d(np.sum(data*data, axis))
    np.sqrt(length, length)
    if axis is not None:
        length = np.expand_dims(length, axis)
    data /= length
    if out is None:
        return data

######################################################################
## 							Matrix Operations 						##
######################################################################

def skew(v):
    if len(v) == 4: v = v[:3]/v[3]
    skv = np.roll(np.roll(np.diag(v.flatten()), 1, 1), -1, 0)
    return skv - skv.T

def r3_X_m(r3a):
	""" convert 3D angular vector in R^(3x1) to SE(3) """

	out = np.eye(4)
	out[0:3,0:3] = rotation_zyx(r3a)
	return out

def v3_X_m(v3a):
	""" convert 3D linear vector in R^(3x1) to SE(3) """

	out = np.eye(4)
	out[0:3,3] = v3a.flatten()
	return out

def mX(a, b, c=None, d=None, e=None):
	""" multiplies 2D array in matrix style """
	
	try:
		if (c is None):
			return np.dot(a,b)
		elif (d is None):
			return np.dot(np.dot(a,b), c)
		elif (e is None):
			return np.dot(np.dot(a,b), np.dot(c,d))
		elif (e is not None):
			return np.dot(np.dot(np.dot(a,b), np.dot(c,d)), e)
	except Exception, e:
		print e
		return None

def mC(a, b):
	""" cross product of two vectors """

	try:
		return (np.cross(a.transpose(),b.transpose())).reshape(3,1)
	except Exception, e:
		print e
		return None

######################################################################
## 						Rotation Operations 						##
######################################################################

def rot_X(q):
    ei = np.eye(3);
    ei[1,1] =  np.cos(q)
    ei[1,2] = -np.sin(q)
    ei[2,1] =  np.sin(q)
    ei[2,2] =  np.cos(q)
    return ei

def rot_Y(q):
    ei = np.eye(3);
    ei[0,0] =  np.cos(q)
    ei[0,2] =  np.sin(q)
    ei[2,0] = -np.sin(q)
    ei[2,2] =  np.cos(q)
    return ei

def rot_Z(q):
    ei = np.eye(3);
    ei[0,0] =  np.cos(q)
    ei[0,1] = -np.sin(q)
    ei[1,0] =  np.sin(q)
    ei[1,1] =  np.cos(q)
    return ei
# print rot_Y(np.deg2rad(30))

def SO3_selection(qsnorm, axis):
    """Return SO3 matrix with only one non-zero component, or close to it"""
    ei = np.zeros((3,3));
    if (axis=='x'):
        ei[0,0] = qsnorm.item(0);  # x selection
    elif (axis=='y'):
        ei[1,1] = qsnorm.item(1);  # y selection
    elif (axis=='z'):
        ei[2,2] = qsnorm.item(2);  # z selection
    
    return ei

def rotation_xyz(qarr):
	rot_x = qarr.item(0);
	rot_y = qarr.item(1);
	rot_z = qarr.item(2);

	rx = np.matrix([ [1.0, .0, .0], [.0, np.cos(rot_x), -np.sin(rot_x)], [.0, np.sin(rot_x), np.cos(rot_x)] ])
	ry = np.matrix([ [np.cos(rot_y), 0.0, np.sin(rot_y)], [.0, 1.0, .0], [-np.sin(rot_y), 0.0, np.cos(rot_y)] ])
	rz = np.matrix([ [np.cos(rot_z), -np.sin(rot_z), 0.0], [np.sin(rot_z), np.cos(rot_z), 0.0],[.0, .0, 1.0] ])

	return np.array(rx*ry*rz)

def rotation_zyx(qarr):
	rot_x = qarr.item(0);
	rot_y = qarr.item(1);
	rot_z = qarr.item(2);

	rx = np.matrix([ [1.0, .0, .0], [.0, np.cos(rot_x), -np.sin(rot_x)], [.0, np.sin(rot_x), np.cos(rot_x)] ])
	ry = np.matrix([ [np.cos(rot_y), 0.0, np.sin(rot_y)], [.0, 1.0, .0], [-np.sin(rot_y), 0.0, np.cos(rot_y)] ])
	rz = np.matrix([ [np.cos(rot_z), -np.sin(rot_z), 0.0], [np.sin(rot_z), np.cos(rot_z), 0.0],[.0, .0, 1.0] ])

	return np.array(rz*ry*rx)

def axisAngle_to_SO3(axis, angle):
    """Return rotation matrix from axis angle vector."""
    skew_K = skew(axis)       # skew symmetric matrix of unit axis of rotation
    return (np.eye(3) + np.sin(angle)*skew_K + (1-np.cos(angle))*np.dot(skew_K,skew_K));      # instantenous rotation in SO(3)

def euler_from_matrix(matrix, axes='sxyz'):
    """ Return Euler angles from rotation matrix for specified axis sequence.

    axes : One of 24 axis sequences as string or encoded tuple

    Note that many Euler angle triplets can describe one matrix.

    >>> R0 = euler_matrix(1, 2, 3, 'syxz')
    >>> al, be, ga = euler_from_matrix(R0, 'syxz')
    >>> R1 = euler_matrix(al, be, ga, 'syxz')
    >>> np.allclose(R0, R1)
    True
    >>> angles = (4*math.pi) * (np.random.random(3) - 0.5)
    >>> for axes in _AXES2TUPLE.keys():
    ...    R0 = euler_matrix(axes=axes, *angles)
    ...    R1 = euler_matrix(axes=axes, *euler_from_matrix(R0, axes))
    ...    if not np.allclose(R0, R1): print(axes, "failed")

    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        _TUPLE2AXES[axes]  # validation
        firstaxis, parity, repetition, frame = axes

    i = firstaxis
    j = _NEXT_AXIS[i+parity]
    k = _NEXT_AXIS[i-parity+1]

    M = np.array(matrix, dtype=np.float64, copy=False)[:3, :3]
    if repetition:
        sy = math.sqrt(M[i, j]*M[i, j] + M[i, k]*M[i, k])
        if sy > _EPS:
            ax = math.atan2( M[i, j],  M[i, k])
            ay = math.atan2( sy,       M[i, i])
            az = math.atan2( M[j, i], -M[k, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2( sy,       M[i, i])
            az = 0.0
    else:
        cy = math.sqrt(M[i, i]*M[i, i] + M[j, i]*M[j, i])
        if cy > _EPS:
            ax = math.atan2( M[k, j],  M[k, k])
            ay = math.atan2(-M[k, i],  cy)
            az = math.atan2( M[j, i],  M[i, i])
        else:
            ax = math.atan2(-M[j, k],  M[j, j])
            ay = math.atan2(-M[k, i],  cy)
            az = 0.0

    if parity:
        ax, ay, az = -ax, -ay, -az
    if frame:
        ax, az = az, ax
    return ax, ay, az

def euler_from_quaternion(quaternion, axes='sxyz'):
	"""Return Euler angles from quaternion for specified axis sequence.

	>>> angles = euler_from_quaternion([0.99810947, 0.06146124, 0, 0])
	>>> np.allclose(angles, [0.123, 0, 0])
	True

	"""
	return euler_from_matrix(quaternion_matrix(quaternion), axes)

def quaternion_matrix(quaternion):
    """Return homogeneous rotation matrix from quaternion.

    >>> M = quaternion_matrix([0.99810947, 0.06146124, 0, 0])
    >>> np.allclose(M, rotation_matrix(0.123, [1, 0, 0]))
    True
    >>> M = quaternion_matrix([1, 0, 0, 0])
    >>> np.allclose(M, np.identity(4))
    True
    >>> M = quaternion_matrix([0, 1, 0, 0])
    >>> np.allclose(M, np.diag([1, -1, -1, 1]))
    True

    """
    q = np.array(quaternion, dtype=np.float64, copy=True)
    n = np.dot(q, q)
    if n < _EPS:
        return np.identity(4)
    q *= math.sqrt(2.0 / n)
    q = np.outer(q, q)
    return np.array([
        [1.0-q[2, 2]-q[3, 3],     q[1, 2]-q[3, 0],     q[1, 3]+q[2, 0], 0.0],
        [    q[1, 2]+q[3, 0], 1.0-q[1, 1]-q[3, 3],     q[2, 3]-q[1, 0], 0.0],
        [    q[1, 3]-q[2, 0],     q[2, 3]+q[1, 0], 1.0-q[1, 1]-q[2, 2], 0.0],
        [                0.0,                 0.0,                 0.0, 1.0]])

def quaternion_matrix_JPL(quaternion):
    q = quaternion.copy()
    q[1:4] = -q[1:4]
    return quaternion_matrix(q)

def rotation_matrix(angle, direction, point=None):
    """ Return matrix to rotate about axis defined by point and direction. """

    sina = math.sin(angle)
    cosa = math.cos(angle)
    direction = unit_vector(direction[:3])
    # rotation matrix around unit vector
    R = np.diag([cosa, cosa, cosa])
    R += np.outer(direction, direction) * (1.0 - cosa)
    direction *= sina
    R += np.array([[ 0.0,         -direction[2],  direction[1]],
                      [ direction[2], 0.0,          -direction[0]],
                      [-direction[1], direction[0],  0.0]])
    M = np.identity(4)
    M[:3, :3] = R
    if point is not None:
        # rotation not around origin
        point = np.array(point[:3], dtype=np.float64, copy=False)
        M[:3, 3] = point - np.dot(R, point)
    return M

def gram_schmidt(e1, e2):
    # simply
    q1 = e1
    # projection of e2 along e1
    proj_vec = (np.dot(e1, e2) / np.dot(e1, e1)) * e1
    # subtract the projection from e2 to get a vector normal to e1
    q2 = e2 - proj_vec
    # obviously
    q3 = np.cross(q1, q2)

    # convert to column vector and normalise
    q1 = q1.reshape((3,1)) / np.linalg.norm(q1)
    q2 = q2.reshape((3,1)) / np.linalg.norm(q2)
    q3 = q3.reshape((3,1)) / np.linalg.norm(q3)
    
    return np.hstack((q1, q2, q3))

######################################################################
##                      Quaternion Operations                       ##
######################################################################

def quaternion_multiply(quaternion1, quaternion0):
    """Return multiplication of two quaternions."""
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return np.array([
        -x1*x0 - y1*y0 - z1*z0 + w1*w0,
        x1*w0 + y1*z0 - z1*y0 + w1*x0,
        -x1*z0 + y1*w0 + z1*x0 + w1*y0,
        x1*y0 - y1*x0 + z1*w0 + w1*z0], dtype=np.float64)

def quaternion_multiply_JPL(q, p):
    # JPL convention
    q4, q1, q2, q3 = q # qr, qx, qy, qz
    p4, p1, p2, p3 = p
    return np.array([
        -q1*p1 - q2*p2 - q3*p3 + q4*p4,
        q4*p1 + q3*p2 - q2*p3 + q1*p4,
        -q3*p1 + q4*p2 + q1*p3 + q2*p4,
        q2*p1 - q1*p2 + q4*p3 + q3*p4], dtype=np.float64)

def quaternion_from_matrix(matrix):
    """Return quaternion from rotation matrix."""

    M = np.array(matrix, dtype=np.float64, copy=False)[:4, :4]
    m00 = M[0, 0]
    m01 = M[0, 1]
    m02 = M[0, 2]
    m10 = M[1, 0]
    m11 = M[1, 1]
    m12 = M[1, 2]
    m20 = M[2, 0]
    m21 = M[2, 1]
    m22 = M[2, 2]
    # symmetric matrix K
    K = np.array([[m00-m11-m22, 0.0,         0.0,         0.0],
                     [m01+m10,     m11-m00-m22, 0.0,         0.0],
                     [m02+m20,     m12+m21,     m22-m00-m11, 0.0],
                     [m21-m12,     m02-m20,     m10-m01,     m00+m11+m22]])
    K /= 3.0
    # quaternion is eigenvector of K that corresponds to largest eigenvalue
    w, V = np.linalg.eigh(K)
    q = V[[3, 0, 1, 2], np.argmax(w)]

    if q[0] < 0.0:
        np.negative(q, q)
    return q

def quaternion_from_matrix_JPL(matrix):
    q = quaternion_from_matrix(matrix)
    q[1:4] = -q[1:4]
    return q

def quaternion_from_euler(ai, aj, ak, axes='sxyz'):
    """Return quaternion from Euler angles and axis sequence.

    ai, aj, ak : Euler's roll, pitch and yaw angles
    axes : One of 24 axis sequences as string or encoded tuple

    >>> q = quaternion_from_euler(1, 2, 3, 'ryxz')
    >>> numpy.allclose(q, [0.435953, 0.310622, -0.718287, 0.444435])
    True

    """
    try:
        firstaxis, parity, repetition, frame = _AXES2TUPLE[axes.lower()]
    except (AttributeError, KeyError):
        _TUPLE2AXES[axes]  # validation
        firstaxis, parity, repetition, frame = axes

    i = firstaxis + 1
    j = _NEXT_AXIS[i+parity-1] + 1
    k = _NEXT_AXIS[i-parity] + 1

    if frame:
        ai, ak = ak, ai
    if parity:
        aj = -aj

    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    if repetition:
        q[0] = cj*(cc - ss)
        q[i] = cj*(cs + sc)
        q[j] = sj*(cc + ss)
        q[k] = sj*(cs - sc)
    else:
        q[0] = cj*cc + sj*ss
        q[i] = cj*sc - sj*cs
        q[j] = cj*ss + sj*cc
        q[k] = cj*cs - sj*sc
    if parity:
        q[j] *= -1.0

    return q

def euler_from_quaternion(quaternion, axes='sxyz'):
    """Return Euler angles from quaternion for specified axis sequence."""
    return euler_from_matrix(quaternion_matrix(quaternion), axes)

def euler_from_quaternion_JPL(quaternion, axes='sxyz'):
    # JPL convention
    q = quaternion.copy()
    q[1:4] = -q[1:4]
    return euler_from_matrix(quaternion_matrix(q), axes)

def quaternion_conjugate(quaternion):

    q = np.array(quaternion, dtype=np.float64, copy=True)
    np.negative(q[1:], q[1:])
    return q

def vec_X_vec_rotation(v1, v2):
    
    out = np.zeros(4)
    dot = np.dot(v1, v2);
    if (dot < -0.999999):
        pass
        tmp = np.cross(np.array([1.,0.,0.]), v1);
        if (np.linalg.norm(tmp) < 0.000001):
            tmp = np.cross(np.array([1.,0.,0.]), v1)
        tmp = tmp/np.linalg.norm(tmp)
        print 'Error in matrix transforms'
        return axisAngle_to_SO3(tmp, math.pi)
    elif (dot > 0.999999):
        out[0] = 0;
        out[1] = 0;
        out[2] = 0;
        out[3] = 1;
        return out
    else:
        tmpvec3 = np.cross(v1, v2);
        out[0] = tmpvec3[0];
        out[1] = tmpvec3[1];
        out[2] = tmpvec3[2];
        out[3] = 1 + dot;
        out = out/np.linalg.norm(out);
        return out
        # return quaternion_matrix(out)[:3,:3]

# print rotation_matrix(np.deg2rad(30),[0, 1, 0])
######################################################################
##                      Constant Parameters                         ##
######################################################################

# epsilon for testing whether a number is close to zero
_EPS = np.finfo(float).eps * 4.0

# axis sequences for Euler angles
_NEXT_AXIS = [1, 2, 0, 1]

# map axes strings to/from tuples of inner axis, parity, repetition, frame
_AXES2TUPLE = {
    'sxyz': (0, 0, 0, 0), 'sxyx': (0, 0, 1, 0), 'sxzy': (0, 1, 0, 0),
    'sxzx': (0, 1, 1, 0), 'syzx': (1, 0, 0, 0), 'syzy': (1, 0, 1, 0),
    'syxz': (1, 1, 0, 0), 'syxy': (1, 1, 1, 0), 'szxy': (2, 0, 0, 0),
    'szxz': (2, 0, 1, 0), 'szyx': (2, 1, 0, 0), 'szyz': (2, 1, 1, 0),
    'rzyx': (0, 0, 0, 1), 'rxyx': (0, 0, 1, 1), 'ryzx': (0, 1, 0, 1),
    'rxzx': (0, 1, 1, 1), 'rxzy': (1, 0, 0, 1), 'ryzy': (1, 0, 1, 1),
    'rzxy': (1, 1, 0, 1), 'ryxy': (1, 1, 1, 1), 'ryxz': (2, 0, 0, 1),
    'rzxz': (2, 0, 1, 1), 'rxyz': (2, 1, 0, 1), 'rzyz': (2, 1, 1, 1)}

_TUPLE2AXES = dict((v, k) for k, v in _AXES2TUPLE.items())


