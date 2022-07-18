'''
* First target checker.
* Has not been tested in a long time. Would not count on it working.
'''

import subprocess
import numpy as np
import matplotlib.pyplot as plt
import pathlib
import os
import sys
import random
import time

# # DIRECTORY MANAGEMENT FOR DARP IMPORT
path = pathlib.Path(__file__).parent.absolute()
path = pathlib.Path(path).parent.absolute()
os.chdir(path)
sys.path.append(str(path))

import DARP_Python_Main as DPM  # pylint: disable=import-error

# Ensures it prints entire arrays when logging instead of going [1 1 1 ... 2 2 2]
np.set_printoptions(threshold=np.inf)

class target_case_checker:
    def __init__(self,Imp=False,maxIter=10000,dcells=30):
        # DIRECTORY MANAGEMENT
        path = pathlib.Path(__file__).parent.absolute()
        # Changing current working directory to be able to find Case**.txt files
        os.chdir(path)
        self.Imp = Imp
        self.maxIter = maxIter
        self.dcells = dcells
    def get_grid(self,file_name):
        FILE = open(file_name,"r")
        self.rows = int(FILE.readline())
        self.cols = int(FILE.readline())
        Grid_string = FILE.readlines()
        self.Grid = np.zeros(self.rows*self.cols,dtype=int)

        el = 0
        for grid_entry in Grid_string:
            for c in grid_entry:
                if (c != " ") and (c != "\n") and (c != "\t"):
                    self.Grid[el] = int(c)
                    el += 1

        self.Grid = self.Grid.reshape([self.rows,self.cols])
        FILE.close()
    def rerun_DARP(self, log_filename, show_grid=False):
        dp = DPM.DARP(self.Grid, self.dcells, self.Imp, log_filename, show_grid,self.maxIter)
        dp.main_DARP()
        self.A = dp.A
        self.n_r = dp.n_r
        self.cols = dp.cols
        self.rows = dp.rows
        self.rip = dp.rip
        self.Ilabel = dp.Ilabel_final
                
if __name__ == "__main__":
    make_plots = True
    
    TCC = target_case_checker(dcells=5)
    TCC.get_grid("TARGET_CASES/Case05-01.txt")
    TCC.rerun_DARP("target_logger.txt",show_grid=make_plots)
    if make_plots == True:
        plt.show()


    