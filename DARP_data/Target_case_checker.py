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

class primMST:
    def __init__(self,A,n_r,rows,cols,rip,Ilabel):
        self.A = A
        self.n_r = n_r
        self.rows = rows
        self.cols = cols
        self.rip = rip
        self.grids = Ilabel

        # Grids: 0 is obstacle, 1 is free space
        # Graphs represent each individual node and which nodes it is connected to
        moves = np.array([[0,1],[1,0],[0,-1],[-1,0]])
        r=0    
        self.free_nodes = np.argwhere(self.grids[r]==1)   
        self.node_list = self.free_nodes 
        self.graph = np.zeros([len(self.free_nodes),len(self.free_nodes)],dtype = int)
        # for node_ind in range(len(self.free_nodes)):
        node_ind = 0
        for node_ind in range(len(self.free_nodes)):
            node = self.free_nodes[node_ind]
            for move in moves:
                row = node[0]+move[0]
                col = node[1]+move[1]
                if(row>=0)and(col>=0)and(row<self.rows)and(col<self.cols):
                    neighbour_node_ind = np.argwhere((self.free_nodes==(row,col)).all(axis=1))
                    # self.free_nodes == (row,col) prints a matrix of the same size as self.free_node, except True in every first column position = row, and True in every econd column position = col (False everywhere else)
                    # (self.free_nodes==(row,col)).all(axis=1) will check in which row there is a True for both the first and second col and return that index. axis=1 means it checks along the columns (thereby stepping through the rows)
                    if len(neighbour_node_ind)>0:
                        self.graph[node_ind][neighbour_node_ind[0][0]]=1 # weights are just 1 for now
                        # self.graph[neighbour_node_ind[0][0]][node_ind]=1
        # Not sure if this is necessary
        for r in range(self.n_r):
            self.grids[r][self.rip[r][0]][self.rip[r][1]] = 2
                
if __name__ == "__main__":
    TCC = target_case_checker()
    TCC.get_grid("TARGET_CASES/Case03.txt")
    TCC.rerun_DARP("target_logger.txt",show_grid=False)
    # plt.show()

    pSTC = primMST(TCC.A,TCC.n_r,TCC.rows,TCC.cols,TCC.rip,TCC.Ilabel)


    