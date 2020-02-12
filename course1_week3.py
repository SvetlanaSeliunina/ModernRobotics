import modern_robotics as mr
import numpy as np
w = np.array([[1],[2],[0.5]])
so = mr.VecToso3(w)
print(so)
wt=np.array([[0, 0.5, -1],[-0.5, 0, 2],[1, -2, 0]])
R = mr.MatrixExp3(wt)
print(R)
r = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
log = mr.MatrixLog3(r)
print(log)
