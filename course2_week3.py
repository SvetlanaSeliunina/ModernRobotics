import modern_robotics as mr
import numpy as np
import math
theta0 = (math.pi/4, math.pi/4, math.pi/4)
Tsd = [[-0.585,-0.811,0,0.076],[0.811,-0.585,0,2.608],[0,0,1,0],[0,0,0,1]]
ev = 0.0001
ew = 0.001
M = [[1,0,0,3],[0,1,0,0],[0,0,1,0],[0,0,0,1]]
S = [[0,0,0],[0,0,0],[1,1,1],[0,0,0],[0,-1,-2],[0,0,0]]
print(mr.IKinSpace(S,M,Tsd,theta0,ew,ev))
