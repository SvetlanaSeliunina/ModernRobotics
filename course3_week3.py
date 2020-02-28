import modern_robotics as mr
import numpy as np
import math
T=5
t=3
s = mr.QuinticTimeScaling(T, t)
print(s)
N=10
Xstart = np.identity(4)
Xend = np.array([[0,0,1,1],[1,0,0,2],[0,1,0,3],[0,0,0,1]])
Tf = 3
screwtraj = mr.ScrewTrajectory(Xstart, Xend, Tf, N, 3)
print(screwtraj[8])
carttraj = mr.CartesianTrajectory(Xstart, Xend, Tf, N, 5)
print(carttraj[8])

