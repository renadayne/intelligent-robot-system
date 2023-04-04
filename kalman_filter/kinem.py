import numpy as np
import random
import math

def A(dS, dTheta, pos_est_T):
    res = np.matrix([
        [1, 0, -dS*math.sin(pos_est_T + dTheta/2)],
        [0, 1, dS*math.cos(pos_est_T + dTheta/2)],
        [0, 0, 1]])
    return res
    
def W(dt, r_wheel, pos_est_T, dTheta, dS, b_wheel):
    res = dt*(r_wheel/2)*np.matrix([[math.cos(pos_est_T + dTheta/2) - (dS/b_wheel)*math.sin(pos_est_T + dTheta/2), math.cos(pos_est_T + dTheta/2) + (dS/b_wheel)*math.sin(pos_est_T + dTheta/2)], 
                                       [math.sin(pos_est_T + dTheta/2) + (dS/b_wheel)*math.cos(pos_est_T + dTheta/2), math.sin(pos_est_T + dTheta/2) - (dS/b_wheel)*math.cos(pos_est_T + dTheta/2)],
                                       [1/b_wheel, -1/b_wheel]])
    return res

class Filter:

    def __init__(self, r, l, R):
        self.r = r
        self.l = l
        self.R = R
        self.P = np.matrix([[0.1, 0, 0],[0, 0.1, 0],[0, 0, 0.1]])
        self.Rref = np.matrix([[0.01*r, 0, 0], [0, 0.01*r, 0], [0, 0, 0.0018*r]])
    
    def setPos(self, px, py, theta):
        self.px = px
        self.py = py
        self.theta = theta
        self.real_tracker = np.matrix([0, 0, px, py, theta], dtype=float)
        self.measurement_tracker = np.matrix([px, py, theta], dtype=float)
        self.estimate = np.matrix([px, py, theta], dtype=float)

    def setV(self, wl, wr):
        self.wl = wl
        self.wr = wr

    def setNoise(self, l_noise, r_interfere):
        self.Q = np.matrix([[l_noise*l_noise, 0], [0, r_interfere*r_interfere]])
        self.wl += l_noise
        self.wr += r_interfere    

    def setDeltaT(self, dT):
        self.dT = dT    

    def appendTracker(self):

        mA = A(self.dT*self.v, self.dT*self.w, self.estimate[len(self.estimate) - 1, 2])
        mW = W(self.dT, self.r, self.estimate[len(self.estimate) - 1, 2], self.dT*self.w, self.dT*self.v, self.l)

        self.P = mA*self.P*mA.transpose() + mW*self.Q*mW.transpose()

        K = self.P/(self.P + self.Rref)
        
        bef_est = np.matrix([self.estimate[len(self.estimate) - 1, 0] + self.dT*self.v*math.cos(self.estimate[len(self.estimate) - 1, 2] + self.w/2),
                             self.estimate[len(self.estimate) - 1, 1] + self.dT*self.v*math.sin(self.estimate[len(self.estimate) - 1, 2] + self.w/2),
                             self.estimate[len(self.estimate) - 1, 2] + self.dT*self.w
                             ])
        
        pos_mes = np.matrix([self.measurement_tracker[len(self.measurement_tracker) - 1, 0], self.measurement_tracker[len(self.measurement_tracker) - 1, 1], self.measurement_tracker[len(self.measurement_tracker) - 1, 2]])

        residual = pos_mes - bef_est
        pos_est = bef_est.transpose() + K*residual.transpose()

        self.P = self.P - K*self.P

        self.estimate = np.append(self.estimate, [[float(pos_est[0]), float(pos_est[1]), float(pos_est[2])]], axis=0)

        self.measurement_tracker = np.append(self.measurement_tracker, [[self.px + random.randint(-1, 1)/80, self.py + random.randint(-1, 1)/80, self.theta + random.randint(-1, 1)/200]], axis=0)

        self.real_tracker = np.append(self.real_tracker,[[self.wl, self.wr, self.px, self.py, self.theta]], axis=0)
        

    def nextPos(self):

        self.w = (self.wr - self.wl)*self.r/self.l
        self.v = (self.wr + self.wl)*self.r/2

        self.px = self.px + self.v*self.dT*math.cos((self.theta + self.theta + self.w*self.dT)/2)
        self.py = self.py + self.v*self.dT*math.sin((self.theta + self.theta + self.w*self.dT)/2)
        self.theta = self.theta + self.w*self.dT
        self.appendTracker()    
   