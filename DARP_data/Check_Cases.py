import subprocess
import numpy as np
import matplotlib.pyplot as plt
import pathlib
import os
import random
import time

## DIRECTORY MANAGEMENT
path = pathlib.Path(__file__).parent.absolute() 
os.chdir(path)  # Changing current working directory to directory this file is in (avoid directory conflict with subprocess) 
print("CURRENT WORKING DIRECTORY:",os.getcwd()) 

file = open("Case01.txt","r")

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
Ilabel = file.readline()
Grid = file.readline()
A = file.readline()

connected_bool = np.zeros(n_r,dtype=bool)
r=0
for c in connected_bool_string:
    if c=='T':
        connected_bool[r] = True
        r+=1
    if c=='F':
        connected_bool[r] = False
        r+=1

# parse grid and Ilabel and A
# steal printing algorithm from DARP_Python_Main

file.close()
