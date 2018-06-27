import eigen_python as ep
import numpy as np
# import compute_com as cc

DIM = 4
M = np.array(ep.compute_com(DIM), order="F")
M.shape= DIM,DIM

print(M) 
print type(M)

# M = np.array(cc.compute_com(4), order="F")