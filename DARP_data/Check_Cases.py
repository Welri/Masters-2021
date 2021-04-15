import subprocess
import numpy as np
import matplotlib.pyplot as plt
import pathlib
import os
import random
import time

# # DIRECTORY MANAGEMENT
# path = pathlib.Path(__file__).parent.absolute()
# # Changing current working directory to directory this file is in (avoid directory conflict with subprocess)
# path = pathlib.Path(path).parent.absolute()
# os.chdir(path)
# print("CURRENT WORKING DIRECTORY:", os.getcwd())

# import DARP_Python_Main


class check_cases:
    def __init__(self):
        # DIRECTORY MANAGEMENT
        path = pathlib.Path(__file__).parent.absolute()
        # Changing current working directory to directory this file is in (avoid directory conflict with subprocess)
        os.chdir(path)
        print("CURRENT WORKING DIRECTORY:", os.getcwd())

    def get_values(self, filename, print=False):
        file = open(filename, "r")

        self.abort = bool(file.readline())
        self.dcells = int(file.readline())
        self.Imp = bool(file.readline())
        self.rows = int(file.readline())
        self.cols = int(file.readline())
        self.n_r = int(file.readline())
        self.es_flag = bool(file.readline())
        self.cc = float(file.readline())
        self.rl = float(file.readline())
        self.max_iter = int(file.readline())
        self.obs = int(file.readline())
        self.discr_achieved = int(file.readline())
        self.iter_achieved = int(file.readline())
        self.time_elapsed = int(file.readline())
        connected_bool_string = file.readline()
        Ilabel_string = file.readline()
        Grid_string = file.readline()
        A_string = file.readline()

        # Extract connected boolean values
        self.connected_bool = np.zeros(self.n_r, dtype=bool)
        r = 0
        for c in connected_bool_string:
            if c == 'T':
                self.connected_bool[r] = True
                r += 1
            if c == 'F':
                self.connected_bool[r] = False
                r += 1

        # Extract environment grid values
        self.Grid = np.zeros(self.rows*self.cols, dtype=int)

        el = 0
        for c in Grid_string:
            if (c != " ") and (c != "\n"):
                self.Grid[el] = int(c)
                el += 1

        self.Grid = self.Grid.reshape(self.rows, self.cols)

        # Extract assignment matrix values
        self.A = np.zeros(self.rows*self.cols, dtype=int)

        if(self.n_r <= 10):
            el = 0
            for c in A_string:
                if (c != " ") and (c != "\n"):
                    self.A[el] = int(c)
                    el += 1
        else:
            el = 0
            iter_var = iter(range(len(A_string)))
            for i in iter_var:
                e = 0
                mult = 1
                value = 0
                c = A_string[i+e]
                while (c != " ") and (c != "\n"):
                    value = value*mult+int(c)
                    mult = 10
                    e = e+1
                    c = A_string[i+e]
                if e > 0:
                    if e > 1:
                        for ee in range(e-1):
                            next(iter_var)
                    self.A[el] = value
                    el += 1

        self.A = self.A.reshape(self.rows, self.cols)

        # Extract Ilabel values
        self.Ilabel = np.zeros(self.n_r*self.rows*self.cols, dtype=int)

        el = 0
        iter_var = iter(range(len(Ilabel_string)))
        for i in iter_var:
            e = 0
            mult = 1
            value = 0
            c = Ilabel_string[i+e]
            while (c != " ") and (c != "\n"):
                value = value*mult+int(c)
                mult = 10
                e = e+1
                c = Ilabel_string[i+e]
            if e > 0:
                if e > 1:
                    for ee in range(e-1):
                        next(iter_var)
                self.Ilabel[el] = value
                el += 1

        self.Ilabel = self.Ilabel.reshape(self.n_r, self.rows, self.cols)

        # Print grid
        if print == True:
            self.print_DARP_graph(self.n_r, self.rows,
                                  self.cols, self.A, self.Grid)

        file.close()

    def print_DARP_graph(self, n_r, rows, cols, A, EG):
        rip = np.argwhere(EG == 2)

        # Prints the DARP divisions
        plt.figure(figsize=(5, 5))
        # Initialize cell colours
        colours = ["C0", "C1", "C2", "C3", "C4", "C5", "C6", "C7", "C8", "C9"]
        c = 0
        colour_assignments = {}
        for i in range(n_r):
            colour_assignments[i] = colours[c]
            c = c + 1
            if c == len(colours)-1:
                c = 0
            colour_assignments[n_r] = "k"

        ripy, ripx = zip(*rip)
        ripy = rows - np.array(ripy)
        plt.plot(ripx, ripy, 'xk', markersize=20)

        # Print Assgnments
        for j in range(rows):
            for i in range(cols):
                x1 = i-0.5
                x2 = i+0.5
                y1 = rows - (j-0.5)
                y2 = rows - (j+0.5)
                if A[j][i] == n_r:
                    plt.fill([x1, x1, x2, x2], [y1, y2, y2, y1], "k")
                else:
                    plt.fill([x1, x1, x2, x2], [y1, y2, y2, y1],
                             colour_assignments[A[j][i]])
        # plt.show()

    # def rerun_DARP(self,log_filename,print_rerun=False):
    #     dp = DARP(self.Grid,self.max_iter,self.dcells,self.cc,self.rl,self.Imp,log_filename,print_rerun)
    #     dp.main_DARP()
checker = check_cases()
checker.get_values("Case01.txt", True)
# checker.get_values("Case02.txt", True)
# checker.get_values("Case03.txt", True)
# checker.get_values("Case04.txt", True)
# checker.get_values("Case05.txt", True)
# checker.get_values("Case06.txt", True)
# checker.get_values("Case07.txt", True)
# checker.get_values("Case08.txt", True)
# checker.get_values("Case09.txt", True)
# checker.get_values("Case10.txt", True)
# checker.get_values("Case11.txt", True)
# checker.get_values("Case12.txt", True)
plt.show()
# checker.rerun_DARP("Checker_Logging.txt",True)

# make it possible to rerun DARP
