#!/usr/bin/env python

import numpy as np
import stabilipy
math = stabilipy.Math()
# from math_tools import Math
# mass, dimension, gravity, radius, force limit, robust sphere (not enforced)
poly = stabilipy.StabilityPolygon(5.0, 2, -9.81, 2., 1., -1, 0.)
# contact position
pos = [[[0., 0., 0.0]], 
		[[1., 0., 0.]],
		[[0., 1., 0.]],
		[[1., 1., 0.]]]

pos = [ [[0.5,   0.2, -0.]],
		[[1.0,  -0.2, -0.]],
		[[-1.0,  0.2, -0.]]]#,
		# [[-1.0, -0.2, -0.]]]
# surface normals
normals = [[[0., 0., 1.]], 
			[[0., 0., 1.]],
			[[0., 0., 1.]]]#,
			# [[0., 0.0, 1.]]] 
axisZ = np.array([[0.0], [0.0], [1.0]])
n4 = np.transpose(np.transpose(math.rpyToRot(1.0,0.0,0.0)).dot(axisZ))
# normals[0] = n4.tolist()
# normals[3] = n4.tolist()
# friction coefficient
mu = 1.0

contacts = [stabilipy.Contact(mu, np.array(p).T,
                              stabilipy.utils.normalize(np.array(n).T))
            for p, n in zip(pos, normals)]
poly.contacts = contacts

# modes: best, iteration, precision
poly.compute(stabilipy.Mode.best, epsilon=1e-3, 
									maxIter=8, 
									solver='cdd',
									plot_error=False,
									plot_init=False,
									plot_step=False,
									record_anim=True,
									plot_direction=False,
									plot_final=False)
