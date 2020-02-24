import modern_robotics as mr
import numpy as np
import math
log = open('log.txt', 'w')
iterates = open('iterates.csv', 'wb')
Blist = np.array([[0, 1, 0, 0.191, 0,   0.817],
                  [0, 0,  1, 0.095, -0.817,   0],
                  [0, 0,  1, 0.095, -0.392, 0],
                  [0, 0, 1, 0.095, 0, 0],
                  [0, -1, 0,  -0.082, 0, 0],
                  [0, 0, 1, 0, 0, 0]]).T
M = np.array([[-1, 0,  0, 0.817],
              [ 0, 0,  1, 0.191],
              [ 0, 1, 0, -0.006],
              [ 0, 0,  0, 1]])
T = np.array([[0, 1,  0,     -0.5],
              [0, 0,  -1,      0.1],
              [-1, 0, 0, 0.1],
              [0, 0,  0,      1]])
thetalist0 = np.array([1.5, 2.5, 3,1,1,1])
eomg = 0.001
ev = 0.0001
def IKinBody(Blist, M, T, thetalist0, eomg, ev):
    thetalist = np.array(thetalist0).copy()
    i = 0
    maxiterations = 20
    Tsb = mr.FKinBody(M, Blist,thetalist)
    Vb = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(Tsb), T)))
    errw = np.linalg.norm([Vb[0], Vb[1], Vb[2]])
    errv = np.linalg.norm([Vb[3], Vb[4], Vb[5]])
    err = errw > eomg \
          or errv > ev
    log.write('\n Iteration ' + str(i) + ': \n')
    log.write('joint vector: \n')
    np.savetxt(log, thetalist)
    log.write('\n end-effector configuration: \n')
    np.savetxt(log, Tsb)
    log.write('\n error  twist: \n')
    np.savetxt(log, Vb)
    log.write('\n angular error magnitude: ' + str(errw) + '\n')
    log.write('linear error magnitude: ' + str(errv) + '\n')
    np.savetxt(iterates, [thetalist], delimiter=',', fmt = '%.6f')
    while err and i < maxiterations:
        thetalist = thetalist \
                    + np.dot(np.linalg.pinv(mr.JacobianBody(Blist, \
                                                         thetalist)), Vb)
        i = i + 1
        Tsb = mr.FKinBody(M, Blist,thetalist)
        Vb = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(Tsb), T)))
        errw = np.linalg.norm([Vb[0], Vb[1], Vb[2]])
        errv = np.linalg.norm([Vb[3], Vb[4], Vb[5]])
        err = errw > eomg \
          or errv > ev
        log.write('\n Iteration ' + str(i) + ': \n')
        log.write('joint vector: \n')
        np.savetxt(log, thetalist)
        log.write('\n end-effector configuration: \n')
        np.savetxt(log, Tsb)
        log.write('\n error  twist: \n')
        np.savetxt(log, Vb)
        log.write('\n angular error magnitude: ' + str(errw) + '\n')
        log.write('linear error magnitude: ' + str(errv) + '\n')
        np.savetxt(iterates, [thetalist], delimiter=',', fmt = '%.6f')
    return (thetalist, not err)
print(IKinBody(Blist, M, T, thetalist0, eomg, ev))
log.close()
iterates.close()
