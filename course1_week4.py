import modern_robotics as mr
import numpy as np
T = np.array([[0,-1,0,3],[1,0,0,0],[0,0,1,1],[0,0,0,1]])
i = mr.TransInv(T)
print(i)
v=np.array([[1],[0],[0],[0],[2],[3]])
se = mr.VecTose3(v)
print(se)
p = (0,0,2)
su = (1,0,0)
h = 1
s = mr.ScrewToAxis(p, su, h)
print(s)
st = np.array([[0,-1.5708,0,2.3562],[1.5708,0,0,-2.3562],[0,0,0,1],[0,0,0,0]])
T = mr.MatrixExp6(st)
print(T)
Tnew = np.array([[0,-1,0,3],[1,0,0,0],[0,0,1,1],[0,0,0,1]])
stnew = mr.MatrixLog6(Tnew)
print(stnew)
