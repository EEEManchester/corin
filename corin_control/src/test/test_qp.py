from numpy import array, dot
from qpsolvers import solve_qp
import numpy as np
import quadprog

M = array([[1., 2., 0.], [-8., 3., 2.], [0., 1., 1.]])
P = dot(M.T, M)  # quick way to build a symmetric matrix
q = dot(array([3., 2., 3.]), M).reshape((3,))
G = array([[1., 2., 1.], [2., 0., 1.], [-1., 2., -1.]])
h = array([3., 2., -2.]).reshape((3,))
A = array([1., 1., 1.])
b = array([1.])

# print "QP solution:", solve_qp(P, q, G, h, A, b, solver='cvxopt')
# print "QP solution:", solve_qp(P, q, G, h, A, b, solver='quadprog')

def quadprog_solve_qp(P, q, G=None, h=None, A=None, b=None):
	qp_G = .5 * (P + P.T)   # make sure P is symmetric
	qp_a = -q

	print np.all(np.linalg.eigvals(qp_G) > 0)

	if A is not None:
		qp_C = -np.vstack([A, G]).T
		qp_b = -np.hstack([b, h])
		meq = A.shape[0]
	else:  # no equality constraint
		qp_C = -G.T
		qp_b = -h
		meq = 0
	


	return quadprog.solve_qp(P, q, qp_C, qp_b, meq)[0]

M = array([[-1., 2., 10.], [-8., 3., 2.], [0., 1., 1.]])
P = dot(M.T, M)
q = -dot(M.T, array([3., 2., 3.]))
G = array([[1., 2., 1.], [2., 0., 1.], [-1., 2., -1.]])
h = array([3., 2., -2.]).reshape((3,))

temp = 0.5*(M + M.T) + np.eye(len(M))*(0.001)

print np.all(np.linalg.eigvals(M) > 0)
print np.all(np.linalg.eigvals(temp) > 0)
print np.all(np.linalg.eigvals(P) > 0)

print np.linalg.eigvals(M)
print np.linalg.eigvals(temp)
print np.linalg.eigvals(P)
# print quadprog_solve_qp(P, q, G, h)
# print solve_qp(P, q, G, h, solver='cvxopt')