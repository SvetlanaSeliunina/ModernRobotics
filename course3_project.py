import modern_robotics as mr
import numpy as np
import math
M01 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.089159], [0, 0, 0, 1]]
M12 = [[0, 0, 1, 0.28], [0, 1, 0, 0.13585], [-1, 0, 0, 0], [0, 0, 0, 1]]
M23 = [[1, 0, 0, 0], [0, 1, 0, -0.1197], [0, 0, 1, 0.395], [0, 0, 0, 1]]
M34 = [[0, 0, 1, 0], [0, 1, 0, 0], [-1, 0, 0, 0.14225], [0, 0, 0, 1]]
M45 = [[1, 0, 0, 0], [0, 1, 0, 0.093], [0, 0, 1, 0], [0, 0, 0, 1]]
M56 = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0.09465], [0, 0, 0, 1]]
M67 = [[1, 0, 0, 0], [0, 0, 1, 0.0823], [0, -1, 0, 0], [0, 0, 0, 1]]
G1 = np.diag([0.010267495893, 0.010267495893,  0.00666, 3.7, 3.7, 3.7])
G2 = np.diag([0.22689067591, 0.22689067591, 0.0151074, 8.393, 8.393, 8.393])
G3 = np.diag([0.049443313556, 0.049443313556, 0.004095, 2.275, 2.275, 2.275])
G4 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G5 = np.diag([0.111172755531, 0.111172755531, 0.21942, 1.219, 1.219, 1.219])
G6 = np.diag([0.0171364731454, 0.0171364731454, 0.033822, 0.1879, 0.1879, 0.1879])
Glist = [G1, G2, G3, G4, G5, G6]
Mlist = [M01, M12, M23, M34, M45, M56, M67] 
Slist = [[0,         0,         0,         0,        0,        0],
         [0,         1,         1,         1,        0,        1],
         [1,         0,         0,         0,       -1,        0],
         [0, -0.089159, -0.089159, -0.089159, -0.10915, 0.005491],
         [0,         0,         0,         0,  0.81725,        0],
         [0,         0,     0.425,   0.81725,        0,  0.81725]]
theta01 = np.array([0, 0, 0, 0, 0, 0]).T
theta02 = np.array([0, -1, 0, 0, 0, 0]).T
dtheta0 = np.array([0, 0, 0, 0, 0, 0]).T
g = np.array([0,0,-9.81]).T
Ftip = np.array([0, 0, 0, 0, 0, 0]).T
tau = np.array([0, 0, 0, 0, 0, 0]).T
N = 1000
t1 = 3
t2 = 5
# simulation 1
simulation1 = open('simulation1.csv', 'wb')
thetalist = theta01.T
dthetalist = dtheta0.T
ddthetalist = []
N1 = N*t1
dt = t1/N1
i = 0
while i < N1:
    if i==0:
        ddthetalist = mr.ForwardDynamics(thetalist.T, dthetalist.T, tau, g, Ftip, Mlist, Glist, Slist)
        thetalist = np.vstack((thetalist, thetalist+dthetalist*dt))
        dthetalist = np.vstack((dthetalist, dthetalist+ddthetalist*dt))
        i = i+1
    elif i==1:
        newddtheta = mr.ForwardDynamics(thetalist[i, :].T, dthetalist[i, :].T, tau, g, Ftip, Mlist, Glist, Slist)
        ddthetalist = np.append([ddthetalist], [newddtheta], axis = 0)
        newtheta = thetalist[i, :]+dthetalist[i, :]*dt
        thetalist = np.append(thetalist, [newtheta], axis =0)
        newdtheta  = dthetalist[i, :]+ddthetalist[i, :]*dt
        dthetalist = np.append(dthetalist, [newdtheta], axis = 0)
        i = i+1
    else:
        newddtheta = mr.ForwardDynamics(thetalist[i, :].T, dthetalist[i, :].T, tau, g, Ftip, Mlist, Glist, Slist)
        ddthetalist = np.append(ddthetalist, [newddtheta], axis = 0)
        newtheta = thetalist[i, :]+dthetalist[i, :]*dt
        thetalist = np.append(thetalist, [newtheta], axis =0)
        newdtheta  = dthetalist[i, :]+ddthetalist[i, :]*dt
        dthetalist = np.append(dthetalist, [newdtheta], axis = 0)
        i = i+1
np.savetxt(simulation1, thetalist, delimiter=',', fmt = '%.6f')
simulation1.close()
# simulation2
simulation2 = open('simulation2.csv', 'wb')
thetalist = theta02.T
dthetalist = dtheta0.T
ddthetalist = []
N2 = N*t2
dt = t2/N2
i = 0
while i < N2:
    if i==0:
        ddthetalist = mr.ForwardDynamics(thetalist.T, dthetalist.T, tau, g, Ftip, Mlist, Glist, Slist)
        thetalist = np.vstack((thetalist, thetalist+dthetalist*dt))
        dthetalist = np.vstack((dthetalist, dthetalist+ddthetalist*dt))
        i = i+1
    elif i==1:
        newddtheta = mr.ForwardDynamics(thetalist[i, :].T, dthetalist[i, :].T, tau, g, Ftip, Mlist, Glist, Slist)
        ddthetalist = np.append([ddthetalist], [newddtheta], axis = 0)
        newtheta = thetalist[i, :]+dthetalist[i, :]*dt
        thetalist = np.append(thetalist, [newtheta], axis =0)
        newdtheta  = dthetalist[i, :]+ddthetalist[i, :]*dt
        dthetalist = np.append(dthetalist, [newdtheta], axis = 0)
        i = i+1
    else:
        newddtheta = mr.ForwardDynamics(thetalist[i, :].T, dthetalist[i, :].T, tau, g, Ftip, Mlist, Glist, Slist)
        ddthetalist = np.append(ddthetalist, [newddtheta], axis = 0)
        newtheta = thetalist[i, :]+dthetalist[i, :]*dt
        thetalist = np.append(thetalist, [newtheta], axis =0)
        newdtheta  = dthetalist[i, :]+ddthetalist[i, :]*dt
        dthetalist = np.append(dthetalist, [newdtheta], axis = 0)
        i = i+1
np.savetxt(simulation2, thetalist, delimiter=',', fmt = '%.6f')
simulation2.close()
