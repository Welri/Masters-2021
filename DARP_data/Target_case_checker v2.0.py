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
    def __init__(self):
        # DIRECTORY MANAGEMENT
        path = pathlib.Path(__file__).parent.absolute()
        # Changing current working directory to be able to find Case**.txt files
        os.chdir(path)
    def get_data(self,file_name):
        FILE = open(file_name,"r")
        self.rows = int(FILE.readline())
        self.cols = int(FILE.readline())
        self.n_r = int(FILE.readline())
        self.cc = float(FILE.readline())
        self.rl = float(FILE.readline())
        self.dcells = int(FILE.readline())
        self.Imp = self.import_bool(FILE.readline())
        Grid_string = FILE.readline()
        rip_string = FILE.readline()
        ripsml_string = FILE.readline()
        ripcont_string = FILE.readline()

        # Environment Grid
        self.Grid = self.string_to_grid(Grid_string,self.rows,self.cols,data_type="int")

        # Large cell RIP coordinates
        self.rip = self.string_to_grid(rip_string,self.n_r,2,data_type="int")

        # Small cell RIP coordinates
        self.rip_sml = self.string_to_grid(ripsml_string,self.n_r,2,data_type="int")

        # Continuous RIP coordinates
        self.rip_cont = self.string_to_grid(ripcont_string,self.n_r,2,data_type="float")

        FILE.close()
    def rerun_DARP(self, file_log = "MAIN_LOGGING", show_grid=False,distance_measure = 0):
        DPM.PRINT_DARP = True
        DPM.PRINT_PATH = True
        DPM.PRINT_TREE = True
        DPM.PATH_COLOR = 'w'
        
        DPM.algorithm_start(recompile=True)
        
        RA = DPM.Run_Algorithm(self.Grid, self.rip, self.dcells, self.Imp, show_grid, dist_meas=distance_measure,log_active=True,log_filename=file_log,target_active=False)
        RA.set_continuous(self.rip_sml,self.rip_cont)
        RA.main()

        # self.A = RA.A
        # self.n_r = RA.n_r
        # self.cols =RA.cols
        # self.rows = RA.rows
        # self.rip = RA.rip
        # self.Ilabel = RA.Ilabel_final
    def import_bool(self, string):
        # Extract boolean variables
        if string[0] == "1" or string[0] == "t" or string[0] == "T":
            return(True)
        elif string[0] == "0" or string[0] == "f" or string[0] == "F":
            return(False)
        else:
            if string[0] == " ":
                for c in string:
                    if c == " ":
                        continue
                    else:
                        return(self.import_bool(c))
            print("ERROR: failed to import boolean value from -> ", string)
            return(-1)       
    def string_to_grid(self,string,rows,cols,data_type="int"):
        if (data_type=="int"):
            Grid = np.zeros(rows*cols,dtype=int)
        elif (data_type=="float"):
            Grid = np.zeros(rows*cols,dtype=float)
        e = 0
        c = 0
        while(c<len(string)):
            if (string[c] == ' ') or (string[c] == '\n') or (string[c] == '\t'):
                c+=1
                continue
            else:
                st = ""
                while((string[c] != " ") and (string[c] != "\n") and (string[c] != "\t")):
                    st = st + string[c]
                    c+=1
                if (data_type == "int"):
                    Grid[e] = int(st)
                elif (data_type == "float"):
                    Grid[e] = float(st)
                e += 1 
        Grid = Grid.reshape(rows, cols)
        return(Grid)

if __name__ == "__main__":
    show_grid = True

    TCC = target_case_checker()
    TCC.get_data("TARGET_CASES_v2.0/Case01.txt")
    TCC.rerun_DARP(show_grid=show_grid,distance_measure=0)
    if (show_grid == True):
        plt.show()
