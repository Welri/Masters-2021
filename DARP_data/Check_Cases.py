import subprocess
import numpy as np
import matplotlib.pyplot as plt
import pathlib
import os
import random
import time

# DIRECTORY MANAGEMENT
path = pathlib.Path(__file__).parent.absolute()
# Changing current working directory to directory this file is in (avoid directory conflict with subprocess)
os.chdir(path)
print("CURRENT WORKING DIRECTORY:", os.getcwd())


def print_DARP_graph(n_r, rows, cols, A, EG):
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
    plt.show()


file = open("Case01.txt", "r")

abort = bool(file.readline())
rows = int(file.readline())
cols = int(file.readline())
n_r = int(file.readline())
es_flag = bool(file.readline())
cc = float(file.readline())
rl = float(file.readline())
max_iter = int(file.readline())
obs = int(file.readline())
discr_achieved = int(file.readline())
iter_achieved = int(file.readline())
time_elapsed = int(file.readline())
connected_bool_string = file.readline()
Ilabel_string = file.readline()
Grid_string = file.readline()
A_string = file.readline()

connected_bool = np.zeros(n_r, dtype=bool)
r = 0
for c in connected_bool_string:
    if c == 'T':
        connected_bool[r] = True
        r += 1
    if c == 'F':
        connected_bool[r] = False
        r += 1

# parse grid and Ilabel and A
# steal printing algorithm from DARP_Python_Main

# Note: Does not account for robots >= 10

Grid = np.zeros(rows*cols, dtype=int)

el = 0
for c in Grid_string:
    if (c != " ") and (c != "\n"):
        Grid[el] = int(c)
        el += 1

Grid = Grid.reshape(rows, cols)

A = np.zeros(rows*cols, dtype=int)

el = 0
for c in A_string:
    if (c != " ") and (c != "\n"):
        A[el] = int(c)
        el += 1

A = A.reshape(rows, cols)

print_DARP_graph(n_r, rows, cols, A, Grid)

file.close()
