# import eigen_python as ep
import numpy as np
# import compute_com as cc

# DIM = 4
# M = np.array(ep.compute_com(DIM), order="F")
# M.shape= DIM,DIM

a = np.array([1,2,3])
b = []
# b.append(a.tolist())
# b.append(list(a))
b.append([1,2,3])
# b = [1,2,3]
# b = lambda b: [item for sublist in l for item in sublist]
c = []
for sublist in b:
    for item in sublist:
        c.append(item)
print c