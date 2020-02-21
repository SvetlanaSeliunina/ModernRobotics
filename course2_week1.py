import modern_robotics as mr
import numpy as np
import math
theta = (-math.pi/2, math.pi/2, math.pi/3, -math.pi/4, 1, math.pi/6)
M = [[1,0,0,3.732],[0,1,0,0],[0,0,1,2.732],[0,0,0,1]]
S = [[0,0,0,0,0,0],[0,1,1,1,0,0],[1,0,0,0,0,1],[0,0,1,-0.732,0,0],[-1,0,0,0,0,-3.732],[0,1,-2.732,3.732,1,0]]
B = [[0,0,0,0,0,0],[0,1,1,1,0,0],[1,0,0,0,0,1],[0,2.732,3.732,2,0,0],[2.732,0,0,0,0,0],[0,-2.732,-1,0,-1,0]]
Tspace = mr.FKinSpace(M,S,theta)
Tbody = mr.FKinBody(M, B, theta)
print(Tspace)
print(Tbody)
