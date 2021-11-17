import matplotlib.pyplot as plt
import numpy as np
import math

class path_planner:
    def __init__(self,wpnt_start,wpnt_end,circle_rad):
        self.r = circle_rad
        Head_start = math.remainder(wpnt_start[1],2*math.pi)
        Head_end = math.remainder(wpnt_end[1],2*math.pi)
        self.PS = wpnt_start[0]
        self.PE = wpnt_end[0]
        self.HS = np.array([math.cos(Head_start),math.sin(Head_start)])
        self.HE = np.array([math.cos(Head_end),math.sin(Head_end)])

        self.CW90 = np.array([[0,-1],[1,0]])
        self.ACW90 = np.array([[0,1],[-1,0]])
    # def RR(self):
    # def RL(self):
    # def LL(self):
    # def LR(self):



V = 30
Phi_max = 26 # max bank angle
r_min = V*V/9.81*np.tan(Phi_max) 
