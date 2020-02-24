import modern_robotics as mr
import numpy as np
import math
theta = (math.pi/2, math.pi/2, 1)
S = [[0,1,0],[0,0,0],[1,0,0],[0,0,0],[0,2,1],[0,0,0]]
B = [[0,-1,0],[1,0,0],[0,0,0],[3,0,0],[0,3,0],[0,0,1]]
Jspace = mr.JacobianSpace(S, theta)
Jbody = mr.JacobianBody(B, theta)
print(Jspace)
print(Jbody)
J = np.array([[-0.105,0,0.006,-0.045,0,0.06,0],[-0.889,0.006,0,-0.844,0.006,0,0],[0,-0.105,0.889,0,0,0,0]])
Jt = np.transpose(J)
A = np.dot(J,Jt)
print(np.linalg.eig(A))
