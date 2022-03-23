import subprocess
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as pat
import matplotlib
from matplotlib.lines import Line2D
import pathlib
import os
import random
import time
import math
from tabulate import tabulate
import Peddle_Planner as pp

# Constants
TREE_COLOR = 'w'
PATH_COLOR = 'k'
PRINT_DARP = True
PRINT_TREE = False
PRINT_PATH = True
PRINT_CIRCLE_CENTRES = False
LINEWIDTH = 0.7
S_MARKERIZE = LINEWIDTH*4
MARKERSIZE=LINEWIDTH*12
TICK_SPACING = 1
FIGURE_TITLE = "DARP Continuous Results"
TARGET_FINDING = True # Does path truncation and target printing
JOIN_REGIONS_FOR_REFUEL = False
REFUEL_TIME = 100

''' Mavic Air 2 Camera (Imagine )
    # Height = 350 # m above ground
    # focal_length = 24 # mm
    # V = 3 
    # H = 4
    # AR = V/H # Aspect ratio (V/H)
    # sensor_width = 6.4 # mm
    # sensor_height = 4.8 # mm
    # FOV_H = Height * sensor_width / focal_length # result in m
    # FOV_V = AR*FOV_H # m
    # px_h = 4000
    # px_w = 3000
    # GSD_h = Height * 100 * (sensor_width/10) / ((focal_length/10) * px_h) # cm/px
    # GSD_w = Height * 100 * (sensor_height/10) / ((focal_length/10) * px_w) # cm/px'''

# GENERAL PARAMETERS
phi_max = 25 # max bank angle in degrees
g_acc = 9.81 # m/s

# CHOSEN VALUES
VEL = 12 # m/s
Height = 120 # m above ground

# Calculating required values from velocity
r_min = VEL**2 / ( g_acc * math.tan(phi_max*math.pi/180) ) # m - Minimum turning radius

# CAMERA PARAMETERS
# Wingtra with Sony RX1R II 
focal_length = 35 # mm
V = 2
H = 3
AR = V/H # Aspect ratio (V/H)
sensor_width = 35.9 # mm
sensor_height = 24 # mm
px_h = 8000
px_w = 5320

# Calculating actual FOV values
FOV_H = Height * sensor_width / focal_length # result in m
FOV_V = AR*FOV_H # m
# Conservatively choose square cell values
# Note: for asymmetric cells FOV_V < FOV_H is a requirement
DISC_H = math.sqrt(2)/(4-math.sqrt(2)) * FOV_H
DISC_V = DISC_H
print("\nDISCRETIZATION SIZE: ", round(DISC_V,2), "X", round(DISC_H,2))
r_max = DISC_V/2
v_max = math.sqrt( r_max * g_acc * math.tan(phi_max*math.pi/180) ) # m/s
GSD_h = Height * 100 * (sensor_width/10) / ((focal_length/10) * px_h) # cm/px
GSD_w = Height * 100 * (sensor_height/10) / ((focal_length/10) * px_w) # cm/px
CT_Overlap = (DISC_V*(FOV_H -DISC_H)) / (DISC_V*DISC_H) # Crosstack overlap

ARC_L = DISC_V/2 - r_min + r_min*np.pi/2 + DISC_H/2 - r_min
FLIGHT_TIME = 30 * 60 # seconds

print("GSD: ",round(GSD_h,2),"Overlap of: ", round(CT_Overlap,2),"\n")

class algorithm_start:
    def __init__(self,recompile=True):
         # DIRECTORY MANAGEMENT
        path = pathlib.Path(__file__).parent.absolute()
        # Changing current working directory to directory this file is in (avoid directory conflict when running subprocesses)
        os.chdir(path)
        # print("CURRENT WORKING DIRECTORY:", os.getcwd())

        # Compile all the Java programs
        if recompile == True:   
            self.compilation_subprocess()
    def compilation_subprocess(self):
        # Runs the OS appropriate script to run the subprocess to compile all the Java files
        if (os.name == 'nt'):
            # print("The current operating system is WINDOWS")
            subprocess.call([r'compiling.bat'])
        elif (os.name == 'posix'):
            # print("The current operating system is UBUNTU")
            subprocess.call("./compiling.sh")
        else:
            print("WARNING: Unrecognised operating system")

# Contains main DARP code, runs the DARP algorithm and runs PrimMST by calling the other class
class Run_Algorithm:
    def __init__(self, EnvironmentGrid, rip, dcells, Imp, show_grid=False,maxIter=10000,cc_vals=np.array([0.1,0.01,0.001,0.0001,0.00001]),rl_vals=np.array([0.01,0.001,0.0001,0.00001]),dist_meas=0, log_filename="MAIN_LOGGING.txt", log_active = False, target_filename = "TARGET_LOG.txt", target_active = False, refuels=0):
        self.Grid = EnvironmentGrid
        self.maxIter = maxIter
        self.dcells = dcells
        self.cc_vals = cc_vals
        self.rl_vals = rl_vals
        self.Imp = Imp
        self.rows = len(self.Grid)
        self.cols = len(self.Grid[0])
        self.rip = np.argwhere(self.Grid == 2) # rip in correct order
        self.rip_temp = rip # rip in incorrect order - same as rip_sml and rip_cont
        self.n_r = len(self.rip) # This is the equivalent number of robots for case where there are refuels
        self.ArrayOfElements = np.zeros(self.n_r, dtype=int)  
        self.abort = False
        self.es_flag = False
        self.runs = 0
        self.show_grid = show_grid
        self.DARP_success = False
        self.log_filename = log_filename
        self.log_active = log_active
        self.target_filename = target_filename
        self.target_active = target_active
        self.total_iterations = 0
        self.distance_measure = dist_meas
        self.refuels = refuels # Number of refuels
        self.nr_og = (int)(self.n_r/(self.refuels+1)) # This is the original number of robots for the case where there are refuels
        # Link original robots with equivalent robots
        self.n_link = np.zeros(self.n_r,dtype=int)
        for r in range(self.n_r):
            self.n_link[r] = math.floor(r/(self.refuels+1))
        self.n_runs = np.zeros(self.n_r,dtype=int)
        val = 0
        for r in range(self.n_r):
            if val==self.refuels:
                self.n_runs[r] = val
                val = 0
            elif val>=0:
                self.n_runs[r] = val
                val += 1
    
    def set_continuous(self,rip_sml,rip_cont,tp_cont=[0,0],start_cont = None):
        self.horizontal = 2*DISC_H*self.cols 
        self.vertical = 2*DISC_V*self.rows
        self.tp_cont = tp_cont # Target initial position
        self.tp_cont[0] = self.vertical - tp_cont[0] # Make it from bottom left corner instead (done like this because environment dimension with grid overlay can be a bit different)
        self.tp_cont[1] = tp_cont[1]
        self.rip_cont = rip_cont # Robot Initial position - continuous
        self.rip_cont_temp = np.zeros([len(self.rip_cont),2])
        self.rip_sml = rip_sml # Robot Initial position - small cells
        self.start_cont = start_cont

        ind = np.zeros([self.n_r],dtype=int)
        rip_sml_temp = np.zeros([self.n_r,2],dtype=int)
        rip_cnt_temp = np.zeros([self.n_r,2])
        # Re-ordering the initial positions as necessary
        for r_0 in range(self.n_r):
            for r_1 in range(self.n_r):
                if(self.rip[r_0][0] == self.rip_temp[r_1][0])and(self.rip[r_0][1] == self.rip_temp[r_1][1]):
                    ind[r_0] = r_1
        for r in range(self.n_r):
            rip_sml_temp[r][0] = self.rip_sml[ind[r]][0]
            rip_sml_temp[r][1] = self.rip_sml[ind[r]][1]
            rip_cnt_temp[r][0] = self.rip_cont[ind[r]][0]
            rip_cnt_temp[r][1] = self.rip_cont[ind[r]][1]
        for r in range(self.n_r):
            self.rip_sml[r][0] = rip_sml_temp[r][0]
            self.rip_sml[r][1] = rip_sml_temp[r][1]
            self.rip_cont_temp[r][0] = rip_cnt_temp[r][0]
            self.rip_cont_temp[r][1] = rip_cnt_temp[r][1]
            self.rip_cont[r][0] = self.vertical - rip_cnt_temp[r][0]
            self.rip_cont[r][1] = rip_cnt_temp[r][1]
         
    def main(self):
        #  DARP SECTION
        timestart = time.time_ns()
        self.enclosed_space_handler()
        self.general_error_handling()
        self.connected_bool = np.zeros(self.n_r, dtype=bool)
        self.Ilabel_final = np.zeros(
            [self.n_r, self.rows, self.cols], dtype=int)
        if self.abort == False:
            for self.cc in self.cc_vals:
                for self.rl in self.rl_vals:
                    self.write_input()
                    self.run_subprocess()
                    self.read_output()
                    self.fairDiv = (self.rows*self.cols - self.obs)/self.n_r
                    self.AOEperc = np.abs((self.ArrayOfElements+1)-self.fairDiv)/self.fairDiv
                    self.maxDiscr = np.max(self.AOEperc)
                    if (self.show_grid == True) and (PRINT_DARP==True):
                        self.print_DARP_graph() # prints a graph for each iteration
                    self.runs += 1
                    self.total_iterations = self.total_iterations + self.iterations
                    print("RUN -","rl: ", self.rl, " cl: ", self.cc, " discrepancy allowed: ", self.dcells/self.fairDiv, "discrepancy achieved: ",self.maxDiscr)
                    if self.DARP_success == True:
                        print("Successfully found solution...")
                        break
                else:
                    continue
                break

        elif self.abort == True:
            # Cancels program if any errors occurred in error handling
            # Calculate the obstacles seen as they aren't returned by Java algorithm
            self.obs = len(np.argwhere(self.Grid == 1))
            print("Aborting Algorithm...")
            return()
        if self.DARP_success == False:
            print("No solution was found...")
        self.time_DARP_total = time.time_ns() - timestart
        
        # Print DARP
        if self.show_grid==True:
            self.cont_DARP_graph() # prints final graph
        
        # PRIM MST SECTION
        timestart = time.time_ns()
        self.primMST()
        self.time_prim = time.time_ns() - timestart

        # Logging
        if(self.log_active == True):
            file_log = open(self.log_filename, "a")
            if(self.abort == False):
                file_log.write(str(self.abort))
                file_log.write(",")
                file_log.write(str(self.dcells))
                file_log.write(",")
                file_log.write(str(self.Imp))
                file_log.write(",")
                file_log.write(str(self.rows))
                file_log.write(",")
                file_log.write(str(self.cols))
                file_log.write(",")
                file_log.write(str(self.n_r))
                file_log.write(",")
                file_log.write(str(self.es_flag))
                file_log.write(",")
                file_log.write(str(self.cc))
                file_log.write(",")
                file_log.write(str(self.rl))
                file_log.write(",")
                file_log.write(str(self.maxIter))
                file_log.write(",")
                file_log.write(str(self.obs))
                file_log.write(",")
                file_log.write(str(self.DARP_success))
                file_log.write(",")
                file_log.write(str(self.discr_achieved))
                file_log.write(",")
                file_log.write(str(self.iterations))
                file_log.write(",")
                file_log.write(str(self.time_DARP_total))
                file_log.write(",")
                file_log.write(str(self.time_prim))
                file_log.write(",")
                file_log.write(str(self.show_grid))
                file_log.write(",")
                AOEstring = str(self.ArrayOfElements)
                AOEstring = AOEstring.replace("\n", '')
                AOEstring = AOEstring.replace("[", '')
                AOEstring = AOEstring.replace("]", '')
                file_log.write(AOEstring)
                file_log.write(",")
                AOEperc_string = str(self.AOEperc)
                AOEperc_string = AOEperc_string.replace("[", '')
                AOEperc_string = AOEperc_string.replace("]", '')
                AOEperc_string = AOEperc_string.replace("\n", '')
                file_log.write(AOEperc_string)
                file_log.write(",")
                file_log.write(str(self.maxDiscr))
                file_log.write(",")
                conBoolstring = str(self.connected_bool)
                conBoolstring = conBoolstring.replace('\n', '')
                conBoolstring = conBoolstring.replace('[', '')
                conBoolstring = conBoolstring.replace(']', '')
                file_log.write(conBoolstring)
                file_log.write(",")
                # ilabelstring = str(self.Ilabel_final)
                # ilabelstring = ilabelstring.replace('\n', '')
                # ilabelstring = ilabelstring.replace('[', '')
                # ilabelstring = ilabelstring.replace(']', '')
                # file_log.write(ilabelstring)
                # file_log.write(",")
                gridstring = str(self.Grid.reshape(1, self.rows*self.cols))
                gridstring = gridstring.replace('\n', '')
                gridstring = gridstring.replace('[', '')
                gridstring = gridstring.replace(']', '')
                file_log.write(gridstring)
                file_log.write(",")
                Astring = str(self.A.reshape(1, self.rows*self.cols))
                Astring = Astring.replace('\n', '')
                Astring = Astring.replace('[', '')
                Astring = Astring.replace(']', '')
                file_log.write(Astring)
                file_log.write(',')
                file_log.write(str(self.runs))
                file_log.write(',')
                file_log.write(str(self.total_iterations))
                file_log.write('\n')
            else:
                file_log.write(str(self.abort))
                file_log.write(",")
                file_log.write(str(self.dcells))
                file_log.write(",")
                file_log.write(str(self.Imp))
                file_log.write(",")
                file_log.write(str(self.rows))
                file_log.write(",")
                file_log.write(str(self.cols))
                file_log.write(",")
                file_log.write(str(self.n_r))
                file_log.write(",")
                file_log.write(str(self.es_flag))
                file_log.write(",")
                file_log.write(str(self.cc))
                file_log.write(",")
                file_log.write(str(self.rl))
                file_log.write(",")
                file_log.write(str(self.maxIter))
                file_log.write(",")
                file_log.write(str(self.obs))
                file_log.write(",")
                file_log.write("None")
                file_log.write(",")
                file_log.write("None")
                file_log.write(",")
                file_log.write("None")
                file_log.write(",")
                file_log.write(str(self.time_DARP_total))
                file_log.write(",")
                file_log.write(str(self.time_prim))
                file_log.write(",")
                file_log.write(str(self.show_grid))
                file_log.write(",")
                file_log.write("None")
                file_log.write(",")
                file_log.write("None")
                file_log.write(",")
                file_log.write("None")
                file_log.write(",")
                file_log.write("None")
                file_log.write(",")
                # file_log.write("None")
                # file_log.write(",")
                gridstring = str(self.Grid.reshape(1, self.rows*self.cols))
                gridstring = gridstring.replace('\n', '')
                gridstring = gridstring.replace('[', '')
                gridstring = gridstring.replace(']', '')
                file_log.write(gridstring)
                file_log.write(",")
                file_log.write("None")
                file_log.write(',')
                file_log.write(str(self.runs))
                file_log.write(',')
                file_log.write(str(self.total_iterations))
                file_log.write('\n')
            file_log.close()

        if(self.target_active == True):
            file_log = open(self.target_filename, "a")
            file_log.write("\n")
            file_log.write(str(self.rows))
            file_log.write("\n")
            file_log.write(str(self.cols))
            file_log.write("\n")
            file_log.write(str(self.n_r))
            file_log.write("\n")
            file_log.write(str(self.cc))
            file_log.write("\n")
            file_log.write(str(self.rl))
            file_log.write("\n")
            file_log.write(str(self.dcells))
            file_log.write("\n")
            file_log.write(str(self.Imp))
            file_log.write("\n")
            file_log.write(str(TARGET_FINDING))
            file_log.write("\n")
            self.tp_cont[0] = self.vertical - self.tp_cont[0]
            targetstring=str(self.tp_cont)
            targetstring = targetstring.replace('\n', '')
            targetstring = targetstring.replace('[', '')
            targetstring = targetstring.replace(']', '')
            file_log.write(targetstring)
            file_log.write("\n")
            gridstring = str(self.Grid.reshape(1, self.rows*self.cols))
            gridstring = gridstring.replace('\n', '')
            gridstring = gridstring.replace('[', '')
            gridstring = gridstring.replace(']', '')
            file_log.write(gridstring)
            file_log.write("\n")
            ripstring = str(self.rip.reshape(1,self.n_r*2))
            ripstring = ripstring.replace('\n', '')
            ripstring = ripstring.replace('[', '')
            ripstring = ripstring.replace(']', '')
            file_log.write(ripstring)
            file_log.write("\n")
            ripsmlstring = str(self.rip_sml.reshape(1,self.n_r*2))
            ripsmlstring = ripsmlstring.replace('\n', '')
            ripsmlstring = ripsmlstring.replace('[', '')
            ripsmlstring = ripsmlstring.replace(']', '')
            file_log.write(ripsmlstring)
            file_log.write("\n")
            ripcontstring = str(self.rip_cont_temp.reshape(1,self.n_r*2))
            ripcontstring = ripcontstring.replace('\n', '')
            ripcontstring = ripcontstring.replace('[', '')
            ripcontstring = ripcontstring.replace(']', '')
            file_log.write(ripcontstring)
            file_log.write("\n")
            file_log.close()
            
    def primMST(self):
        # try:
            # Run MST algorithm
            pMST = Prim_MST_maker(self.A,self.n_r,self.rows,self.cols,self.rip,self.Ilabel_final,self.rip_cont,self.rip_sml,self.tp_cont,self.n_link,self.n_runs,self.refuels,self.nr_og,self.start_cont)
            self.schedule = np.zeros([self.n_r,6])
            # Scheduling protocol
            if(self.refuels > 0):
                r_append = 0 # The waypoint, time and distance arrays get appended in a different order, tracked here
                for run in range(self.refuels+1):
                    take_off_total = 0 # Accumulated take-off times          
                    # Step through a specific run for each of the original robots
                    # One run must complete for all robots before the first robot can take-off again for the next run
                    for r_og in range(self.nr_og):
                        r = r_og*(self.refuels+1)+run
                        if(run == 0):
                            self.schedule[r][0] = take_off_total # Start time
                            take_off_total = take_off_total + pMST.TO_time[r] # Add take-off time to total - this represents the time after take-off
                            self.schedule[r][1] = take_off_total # Time after take-off
                            pMST.waypoint_final_generation(pMST.wpnts_cont_list[r],pMST.wpnts_class_list[r],r,take_off_total) # Start time is end of take-off
                            flight_time = pMST.time_cumulative_list[r_append][-1] # Time after take-off and flight
                            self.schedule[r][2] = flight_time # Time after flight
                            if(r_og == 0):
                                # Wait time is zero for first robot - it lands first
                                self.schedule[r][3] = flight_time  # Time after flight and no wait
                                landing_time = self.schedule[r][3] + pMST.LD_time[r] # Time after take-off, flight and landing
                                self.schedule[r][4] = landing_time # Time after landing
                                self.schedule[r][5] = landing_time + REFUEL_TIME # Time after refuel
                            else:
                                # previous r_og
                                r_prev = (r_og-1)*(self.refuels+1)+run
                                landing_time_prev = self.schedule[r_prev][4]
                                if(flight_time < landing_time_prev):
                                    # If current robot finishes flight before previous one finishes landing, wait time incurred
                                    wait_time = landing_time_prev - flight_time
                                    # Wait time has to be multiple of r_min circumference
                                    circ = 2*np.pi*(r_min)/VEL
                                    multiple = math.ceil(wait_time/circ)
                                    wait_time = multiple*circ
                                    self.schedule[r][3] = flight_time + wait_time # Time after flight and wait time
                                else:
                                    # wait time is 0
                                    self.schedule[r][3] = flight_time # time after flight and no wait
                                landing_time = self.schedule[r][3] + pMST.LD_time[r] # Time after take-off, flight, wait and landing
                                self.schedule[r][4] = landing_time # time after landing
                                self.schedule[r][5] = landing_time + REFUEL_TIME # Time after refuel
                        else:
                            if(r_og==0):
                                # Current robot, previous run - time after refuel
                                r_previous_run = r_og*(self.refuels+1)+(run-1)
                                refuel_end = self.schedule[r_previous_run][5] # Time after r_og finishes previous refuel
                                # Final robot, previous run - time after land
                                rf_previous_run = (self.nr_og-1)*(self.refuels+1)+(run-1)
                                land_end_prev = self.schedule[rf_previous_run][4] # Time after previous robot lands
                                # Starting time is the landing time of previous robot, unless refuel takes longer than that
                                self.schedule[r][0] = max(refuel_end,land_end_prev)
                                take_off_total = self.schedule[r][0] + pMST.TO_time[r]
                                self.schedule[r][1] = take_off_total # Time after take-off of first robot
                            else:
                                # Time after take-off of previous robot
                                self.schedule[r][0] = take_off_total
                                # Time after refuel of current robot in previous run
                                r_previous_run = r_og*(self.refuels+1)+(run-1)
                                refuel_end = self.schedule[r_previous_run][5] # Time after r_og finishes previous refuel
                                # Check if it has finished refuelling and adjust take-off if it hasn't
                                self.schedule[r][0] = max(self.schedule[r][0],refuel_end)
                                take_off_total = self.schedule[r][0] + pMST.TO_time[r] # Time after take-off
                                self.schedule[r][1] = take_off_total
                            pMST.waypoint_final_generation(pMST.wpnts_cont_list[r],pMST.wpnts_class_list[r],r,take_off_total)
                            flight_time = pMST.time_cumulative_list[r_append][-1] # Time after take-off and flight
                            self.schedule[r][2] = flight_time # Time after flight
                            if(r_og == 0):
                                # Wait time is zero - first robot to land (and take-off)
                                self.schedule[r][3] = flight_time # Time after flight and no wait
                                landing_time = flight_time + pMST.LD_time[r] # Time after take-off, flight and landing
                                self.schedule[r][4] = landing_time # Time after landing
                                if (run < self.refuels):
                                    # Not the last run
                                    self.schedule[r][5] = landing_time + REFUEL_TIME
                                else:
                                    # No refuel time
                                    self.schedule[r][5] = landing_time
                            else:
                                r_prev = (r_og-1)*(self.refuels)+run
                                landing_time_prev = self.schedule[r_prev][4]
                                if(flight_time < landing_time_prev):
                                    # If current robot finishes flight before previous one finishes landing
                                    wait_time = landing_time_prev - flight_time
                                    # Wait time has to be multiple of r_min circumference
                                    circ = 2*np.pi*(r_min)/VEL
                                    multiple = math.ceil(wait_time/circ)
                                    wait_time = multiple*circ
                                    self.schedule[r][3] = flight_time + wait_time # Time after flight and wait time
                                else:
                                    self.schedule[r][3] = flight_time # Time after flight and no wait
                                landing_time = self.schedule[r][3] + pMST.LD_time[r] # Time after take-off, flight, wait and landing
                                self.schedule[r][4] = landing_time
                                if (run < self.refuels):
                                    # Not the last run
                                    self.schedule[r][5] = landing_time + REFUEL_TIME
                                else:
                                    # No refuel time
                                    self.schedule[r][5] = landing_time
                        r_append += 1 
            elif(self.refuels == 0):
                for r in range(self.n_r):
                    pMST.waypoint_final_generation(pMST.wpnts_cont_list[r],pMST.wpnts_class_list[r],r,0)

            print("\nALLOWABLE FLIGHT TIME WITH FUEL CONSTRAINTS: ", FLIGHT_TIME )
            
            # Print STC paths on DARP plot
            if (self.show_grid == True):
                for r in range(self.n_r):
                    # pMST.print_graph(pMST.free_nodes_list[r],pMST.parents_list[r],self.ax,r,time_end=pMST.TIME_BREAK)
                    pMST.print_graph(pMST.free_nodes_list[r],pMST.parents_list[r],self.ax,r)

            # Print relevant data in table
            if (self.refuels > 0):
                data = [["Robot","Refuel","OG Robot","Take-off","Flight","Wait","Landing","Total Time","Time Limit","Time Diff","Distance","Rotations"]]
                for r in range(self.n_r):
                    run = self.n_runs[r]
                    r_og = self.n_link[r]
                    take_off = pMST.TO_time[r]
                    landing = pMST.LD_time[r]
                    wait = self.schedule[r][3] - self.schedule[r][2]
                    flight_time = pMST.time_totals[r] # This is still added to an array using original r value, not appended to a list like cumulative times etc.
                    total_time = take_off + flight_time + wait + landing
                    rot_ach = pMST.rotations[r]
                    dist_ach = pMST.dist_totals[r]
                    data.append([r,run,r_og,round(take_off,1),round(flight_time,1),round(wait,1),round(landing,1),round(total_time,1),round(FLIGHT_TIME,1),round(FLIGHT_TIME - total_time,1),round(dist_ach,1),round(rot_ach,0)])
            else:
                data = [["Robot","Total Time","Time Limit","Time Diff","Distance","Rotations"]]
                for r in range(self.n_r):
                    total_time = pMST.time_totals[pMST.r_append[r]]
                    rot_ach = pMST.rotations[r]
                    dist_ach = pMST.dist_totals[r]
                    data.append([r,round(total_time,1),round(FLIGHT_TIME,1),round(FLIGHT_TIME - total_time,1),round(dist_ach,1),round(rot_ach,0)])
            print(tabulate(data))
            
            # Print schedules in data table
            if (self.refuels > 0):
                data = [['Robot','Refuel','OG Robot','Start Time','After Take-off','After Flight','After Wait','After Landing','After Refuel']]
                for run in range(self.refuels+1):
                    for r_og in range(self.nr_og):
                        r = r_og*(self.refuels+1)+run
                        l = list()
                        l.append(r_og*(self.refuels+1)+run)
                        l.append(run)
                        l.append(r_og)
                        for i in range(6):
                            l.append(round(self.schedule[r][i],1))
                        data.append(l)
                print(tabulate(data))

            # Print schedules graph
            if (self.refuels > 0):
                plt.rc('font', size=12)
                plt.rc('axes', titlesize=15)
                fig,ax = plt.subplots(figsize=(12,1.5*self.nr_og))
                ax.set_yticks(np.arange(0,self.nr_og))
                ax.set_xticks(np.linspace(0,np.max(self.schedule),15),minor=False)
                ax.set_xticks(np.linspace(0,np.max(self.schedule),(15-1)*5+1),minor=True)
                ax.invert_yaxis()
                ax.set_ylabel("Robot")
                ax.set_xlabel("Time in seconds")
                ax.set_title("Flight Schedule")
                legend_elements = [ Line2D([0], [0], color="C0",alpha=0.5, lw=4, label='Taking off'),
                                    Line2D([0], [0], color="C1",alpha=0.5, lw=4, label='Flying'),
                                    Line2D([0], [0], color="C2",alpha=0.5, lw=4, label='Waiting'),
                                    Line2D([0], [0], color="C3",alpha=0.5, lw=4, label='Landing'),
                                    Line2D([0], [0], color="C4",alpha=0.5, lw=4, label='Refuelling')]
                
                plt.tight_layout()
                t_colours = ["C0","C1","C2","C3","C4"]
                # Plot target finding time
                plt.plot(np.array([pMST.TIME_BREAK,pMST.TIME_BREAK]),np.array([-0.6,self.nr_og-0.4]),'k',linestyle='dashed',linewidth=1)
                plt.plot(pMST.TIME_BREAK,self.n_link[pMST.TARGET_CELL[0]],'xk',markersize = int(MARKERSIZE))
                # Plot schedules
                for r_og in range(self.nr_og):
                    # Plot per robot
                    max_t = 0
                    min_t = np.inf
                    for run in range(self.refuels+1):
                        r = r_og*(self.refuels+1)+run
                        time_points = self.schedule[r]
                        if(np.max(time_points) > max_t):
                            max_t = np.max(time_points)
                        if(np.min(time_points) < min_t):
                            min_t = np.min(time_points)
                        # Plot starting line
                        t_curr = time_points[0]
                        plt.plot(np.array([t_curr,t_curr]),np.array([r_og-0.5,r_og+0.5]),'k',linewidth = 0.5)
                        # Plot other time lines and boxes
                        for t in range(1,6):
                            t_prev = time_points[t-1]
                            t_curr = time_points[t]
                            plt.plot(np.array([t_curr,t_curr]),np.array([r_og-0.5,r_og+0.5]),'k',linewidth = 0.5)
                            plt.fill([t_prev,t_curr,t_curr,t_prev],[r_og-0.5,r_og-0.5,r_og+0.5,r_og+0.5],t_colours[t-1],alpha=0.5)
                    plt.plot([min_t,max_t],[r_og+0.5,r_og+0.5],'k',linewidth=0.5)
                    plt.plot([min_t,max_t],[r_og-0.5,r_og-0.5],'k',linewidth=0.5)
                val = max_t*0.04
                ax.set_xlim(-val,max_t+val*5)
                # plt.grid(which='major',axis='x', color='k',linewidth=0.1)
                # plt.grid(which='minor',axis='x', color='k',linestyle='dotted',linewidth=0.1)
                # plt.xticks(rotation=90)
                ax.legend(handles = legend_elements,loc="best")
        # except:
        #     print("Prim algorithm failed to implement...")
    
    def enclosed_space_handler(self):
        # Enclosed spaces (unreachable areas) are classified as obstacles
        ES = enclosed_space_check(
            self.n_r, self.rows, self.cols, self.Grid, self.rip)
        if ES.max_label > 1:
            self.es_flag = True
            print(
                "WARNING: Automatic removal of enclosed space (it is considered an obstacle) ....")
            self.label_matrix = ES.final_labels
            inds = np.argwhere(self.label_matrix > 1)
            temp = False
            for ind in inds:
                if self.Grid[ind[0]][ind[1]] == 2:
                    print("WARNING: Automatic removal of robot in enclosed space...")
                    self.Grid[ind[0]][ind[1]] = 1
                    temp = True
                else:
                    self.Grid[ind[0]][ind[1]] = 1
            if temp == True:
                self.rip = np.argwhere(self.Grid == 2)
                self.n_r = len(self.rip)

    def general_error_handling(self):
        if(self.n_r < 1):
            print("WARNING: No Robot Initial Positions Given....\n")
            self.abort = True
        if(self.dcells < 1):
            print("WARNING: Cell Discrepancy Needs to be a Positive Integer...\n")
            self.abort = True
        if(self.maxIter < 1):
            print("WARNING: Maximum Iterations Needs to be a Positive Integer...\n")
            self.abort = True
        if(self.n_r > self.rows*self.cols):
            print("WARNING: More robots than available cells...\n")
            self.abort = True
        if(self.rows*self.cols == 0):
            print("WARNING: One of the environment array dimensions is zero...\n")
            self.abort = True
        if(r_min>r_max):
            print("WARNING: minimum turning radius is too large.\nOPTIONS:\n1. Increase HEIGHT to increase FOV (which is directly related to r_max)\n3. Choose a camera with a larger FOV\n4. Reduce VELOCITY.\n")
            self.abort = True
        if(GSD_h > 4.0):
            print("WARNING: Ground Sampling Distance is too high for reasonable human detection.\nOPTIONS:\n1. Choose camera with higher RESOLUTION\n2. Reduce flying HEIGHT\n3. Choose a camera with a lower ratio of sensor size to focal length.\n")
            # self.abort = True
        if(VEL>v_max):
            print("WARNING: velocity is too high. Cannot make turning radius and have complete coverage.\n")
    
    def write_input(self):
        # Writes relevant inputs for java code to file
        # print(pathlib.Path("Input.txt").absolute())
        # file_in = open("DARP_Java/Input.txt", "w")
        file_in = open("Input.txt", "w")
        file_in.write(str(self.n_r))
        file_in.write('\n')
        file_in.write(str(self.rows))
        file_in.write('\n')
        file_in.write(str(self.cols))
        file_in.write('\n')
        file_in.write(str(self.maxIter))
        file_in.write('\n')
        file_in.write(str(self.dcells))
        file_in.write('\n')
        file_in.write(str(self.cc))
        file_in.write('\n')
        file_in.write(str(self.rl))
        file_in.write('\n')
        file_in.write(str(self.distance_measure))
        file_in.write('\n')
        for i in range(self.rows):
            for j in range(self.cols):
                file_in.write(str(self.Grid[i][j]))
                file_in.write('\n')
        file_in.write(str(self.Imp))
        file_in.close()

    def run_subprocess(self):
        # Runs the OS appropriate script to run the Java program for the DARP algorithm
        # subprocess.call([r'DARP_Java\Run_Java.bat'])
        # print(pathlib.Path('Run_Java.bat').absolute())
        if (os.name == 'nt'):
            # print("The current operating system is WINDOWS")
            subprocess.call([r'Run_Java.bat'])
        elif (os.name == 'posix'):
            # print("The current operating system is UBUNTU")
            subprocess.call("./Run_Java.sh")
        else:
            print("WARNING: Unrecognised operating system")

    def read_output(self):
        # Read file containing outputs that were wrote to file by Java file
        self.A = np.zeros([self.rows, self.cols], dtype=int)
        # print(pathlib.Path("Output_A.txt").absolute())
        # file_out = open("DARP_Java/Output_A.txt", "r")
        file_out = open("Output.txt", "r")
        # Extracting variables from Output file generated by Java
        for i in range(self.rows):
            for j in range(self.cols):
                self.A[i][j] = int(file_out.readline())
        self.discr_achieved = int(file_out.readline())
        self.DARP_success = self.import_bool(file_out.readline())
        self.obs = int(file_out.readline())
        self.iterations = int(file_out.readline())
        for r in range(self.n_r):
            self.ArrayOfElements[r] = int(file_out.readline())
        for r in range(self.n_r):
            self.connected_bool[r] = self.import_bool(file_out.readline())
        for r in range(self.n_r):
            for i in range(self.rows):
                for j in range(self.cols):
                    self.Ilabel_final[r][i][j] = int(file_out.readline())
        file_out.close()

    def print_DARP_graph(self):
        plt.rc('font', size=12)
        plt.rc('axes', titlesize=15) 

        # Prints the DARP divisions
        fig,ax = plt.subplots(figsize=(8, 8))

        # Initialize cell colours
        # TODO: Somewhat redundant since I redo this in the cont function
        colours = ["C0", "C1", "C2", "C3", "C4", "C5", "C6", "C7", "C8", "C9"]
        c = 0
        colour_assignments = {}
        for i in range(self.n_r):
            colour_assignments[i] = colours[c]
            c = c + 1
            if c == len(colours)-1:
                c = 0
            colour_assignments[self.n_r] = "k"

        # Print RIP
        for r in range(self.n_r):
            ripy = self.rows*2 - self.rip_sml[r][0] - 1
            ripx = self.rip_sml[r][1]
            plt.plot(ripx,ripy,'.k', markersize=MARKERSIZE)

        # REMOVE AXES ENTIRELY
        # ax.axes.xaxis.set_visible(False)
        # ax.axes.yaxis.set_visible(False)

        # USED THIS ONCE TO GET TICKS AT THE SMALL CELL SENTRES
        # ax.set_xticks(np.arange(0, self.cols*2, step=TICK_SPACING),minor=True)
        # ax.set_yticks(np.arange(0, self.rows*2, step=TICK_SPACING),minor=True)
        
        ax.set_xticks(np.arange(-0.5, self.cols*2+0.5, step=2),minor=False)
        ax.set_yticks(np.arange(-0.5, self.rows*2+0.5, step=2),minor=False)
        
        plt.xticks(rotation=90)
    
        xticks = list(map(str,np.arange(0, self.cols+1, step=1)))
        yticks = list(map(str,np.arange(0, self.rows+1, step=1)))
        ax.set_xticklabels(xticks)
        ax.set_yticklabels(yticks)

        plt.grid(which='major',axis='both', color='k')

        # Print Assgnments
        for j in range(self.rows):
            for i in range(self.cols):
                x1 = (i-0.5)*2 + 0.5
                x2 = (i+0.5)*2 + 0.5
                y1 = (self.rows - (j-0.5) - 1)*2 + 0.5
                y2 = (self.rows - (j+0.5) - 1)*2 + 0.5
                if self.A[j][i] == self.n_r:
                    plt.fill( [x1, x1, x2, x2], [y1, y2, y2, y1], "k", alpha=0.75 )
                else:
                    if(JOIN_REGIONS_FOR_REFUEL):
                        plt.fill( [x1, x1, x2, x2], [y1, y2, y2, y1], colour_assignments[self.n_link[self.A[j][i]]], alpha=0.75)
                    else:
                        plt.fill( [x1, x1, x2, x2], [y1, y2, y2, y1], colour_assignments[self.A[j][i]], alpha=0.75)
        plt.title("DARP Results")
    
    def cont_DARP_graph(self):
        plt.rc('font', size=12)
        plt.rc('axes', titlesize=15) 

        # Prints the DARP divisions
        fig,self.ax = plt.subplots(figsize=(8, 8))

        # Initialize cell colours
        # colours = ["C0", "C1", "C2", "C3", "C4", "C5", "C6", "C7", "C8", "C9","C10"]
        # colours = ['xkcd:nice blue','xkcd:dusty orange','xkcd:kelly green','xkcd:red','xkcd:brownish','xkcd:carnation pink','xkcd:medium grey','xkcd:gold','xkcd:turquoise blue','xkcd:purple','xkcd:lightblue','xkcd:lavender','xkcd:salmon','xkcd:magenta','xkcd:olive','xkcd:silver']
        # 'dimgray','gray','darkgray','silver','lightgray','gainsboro','whitesmoke','lightslategray','slategray','teal','cadetblue','skyblue','steelblue','slategray','cornflowerblue','forestgreen','mediumseagreen','mediumaquamarine','lightseagreen'
        # colours = ['salmon','firebrick','goldenrod','olivedrab']
        # colours = ['xkcd:purple','xkcd:green','xkcd:magenta','xkcd:blue','xkcd:pink','xkcd:pistachio','xkcd:maroon','xkcd:light blue','xkcd:light pink','xkcd:lavender','xkcd:red','xkcd:orange','xkcd:sand yellow','xkcd:warm brown','xkcd:grey','xkcd:teal']
        colours = ["C0", "C1", "C2", "C3", "C4", "C5", "C6", "C7", 'xkcd:maize', "C9",'xkcd:eggshell','xkcd:light blue','xkcd:light orange','xkcd:light green','xkcd:light lilac','xkcd:light pink']
        c = 0
        colour_assignments = {}
        for i in range(self.n_r):
            colour_assignments[i] = colours[c]
            c = c + 1
            if c == len(colours):
                c = 0
            colour_assignments[self.n_r] = "k"

        # Ticks and Grid
        self.ax.set_xticks(np.arange(0, (self.cols*2+1)*DISC_H, step=2*DISC_H),minor=False) # Large cell grid-lines
        self.ax.set_yticks(np.arange(0, (self.rows*2+1)*DISC_V, step=2*DISC_V),minor=False)
        self.ax.set_xticks(np.arange(DISC_H, (self.cols*2-0.5)*DISC_H, step=2*DISC_H),minor=True) # Small cell grid_lines
        self.ax.set_yticks(np.arange(DISC_V, (self.rows*2-0.5)*DISC_V, step=2*DISC_V),minor=True)
        plt.xticks(rotation=90)
        plt.grid(which='major',axis='both', color='k',linewidth=0.3)
        plt.grid(which='minor',axis='both',color='k',linewidth=0.3,linestyle=':')

        # Note: Robot initial position are printing in the prim algorithm

        # Print Assgnments
        for j in range(self.rows):
            for i in range(self.cols):
                x1 = ( (i-0.5)*2 + 0.5 + 0.5 )*DISC_H
                x2 = ( (i+0.5)*2 + 0.5 + 0.5 )*DISC_H
                y1 = ( (self.rows - (j-0.5) - 1)*2 + 0.5 + 0.5 )*DISC_V
                y2 = ( (self.rows - (j+0.5) - 1)*2 + 0.5 + 0.5 )*DISC_V
                if self.A[j][i] == self.n_r:
                    plt.fill([x1, x1, x2, x2], [y1, y2, y2, y1], "k", alpha=0.75)
                else:
                    if(JOIN_REGIONS_FOR_REFUEL):
                        plt.fill([x1, x1, x2, x2], [y1, y2, y2, y1], colour_assignments[self.n_link[self.A[j][i]]],alpha=0.75)
                    else:
                        plt.fill([x1, x1, x2, x2], [y1, y2, y2, y1], colour_assignments[self.A[j][i]],alpha=0.75)

        plt.title(FIGURE_TITLE)

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

# DARP related
class enclosed_space_check:
    def __init__(self, n_r, n_rows, n_cols, EnvironmentGrid, rip):
        self.n_r = n_r
        self.n_rows = n_rows
        self.n_cols = n_cols
        self.binary_grid = np.copy(EnvironmentGrid)
        self.rip = rip
        self.create_binary_grid()
        self.final_labels = self.compact_labeling()

    def create_binary_grid(self):
        for ind in self.rip:
            self.binary_grid[ind[0]][ind[1]] = 0
        indices_one = self.binary_grid == 1
        indices_zero = self.binary_grid == 0
        self.binary_grid[indices_one] = 0
        self.binary_grid[indices_zero] = 1

    def transform2Dto1D(self, array2D):
        length = len(array2D)*len(array2D[0])
        array1D = np.reshape(array2D, length)
        return(array1D)

    def labeling(self, bin1D):
        rst = np.zeros([self.n_rows*self.n_cols], dtype=int)
        self.parent = np.zeros([self.MAX_LABELS], dtype=int)
        self.labels = np.zeros([self.MAX_LABELS])

        next_region = 1
        for i in range(self.n_rows):  # height
            for j in range(self.n_cols):  # width
                if (bin1D[i*self.n_cols+j] == 0):
                    continue
                k = 0
                connected = False

                # Check if connected to the left
                if ((j > 0) and (bin1D[i*self.n_cols+j-1] == bin1D[i*self.n_cols+j])):
                    k = rst[i*self.n_cols+j-1]
                    connected = True
                # Check if connected to the top
                if ((i > 0) and (bin1D[(i-1)*self.n_cols+j] == bin1D[i*self.n_cols+j]) and ((connected == False) or (bin1D[(i-1)*self.n_cols+j] < k))):
                    k = rst[(i-1)*self.n_cols+j]
                    connected = True
                if(connected == False):
                    k = next_region
                    next_region += 1

                rst[i*self.n_cols+j] = k
                if ((j > 0) and (bin1D[i*self.n_cols+j-1] == bin1D[i*self.n_cols+j]) and rst[i*self.n_cols+j-1] != k):
                    self.uf_union(k, rst[i*self.n_cols+j-1])
                if ((i > 0) and (bin1D[(i-1)*self.n_cols+j] == bin1D[i*self.n_cols+j]) and rst[(i-1)*self.n_cols+j] != k):
                    self.uf_union(k, rst[(i-1)*self.n_cols+j])

        self.next_label = 1
        for ind in range(self.n_cols*self.n_rows):
            if ((bin1D[ind] != 0)):
                rst[ind] = self.uf_find(rst[ind])
        self.next_label -= 1
        return(rst)

    def compact_labeling(self):
        bin_1D = self.transform2Dto1D(self.binary_grid)
        self.MAX_LABELS = self.n_rows * self.n_cols

        # Label the different regions
        label1D = self.labeling(bin_1D)
        self.max_label = self.next_label

        stat = np.zeros(self.max_label+1)
        for l in range(0, self.max_label+1):
            stat[l] = len(np.argwhere(label1D == l))
        if sum(stat) != self.n_cols*self.n_rows:
            print("WARNING: labelling error - line 63-69")
        stat[0] = 0

        counter = 1
        for l in range(self.max_label+1):
            if stat[l] != 0:
                stat[l] = counter
                counter += 1

        # Seems kind of redundant - need to see if it is really useful
        if self.max_label != counter - 1:
            print("Line 81 activated")
            self.max_label == counter - 1
            for ind in range(len(bin_1D)):
                label1D[ind] = stat[label1D[ind]]

        label2D = np.reshape(label1D, [self.n_rows, self.n_cols])
        return(label2D)

    def uf_union(self, a, b):
        a = int(a)
        b = int(b)
        while (self.parent[a] > 0):
            a = self.parent[a]
        while (self.parent[b] > 0):
            b = self.parent[b]
        if (a != b):
            if (a > b):
                self.parent[a] = b
            else:
                self.parent[b] = a

    def uf_find(self, r):
        while (self.parent[r] > 0):
            r = self.parent[r]
        if (self.labels[r] == 0):
            self.labels[r] = self.next_label
            self.next_label += 1
        return self.labels[r]

# Contains main PrimMST code - run from "Run_Algorithm"
class Prim_MST_maker:
    def __init__(self,A,n_r,rows,cols,rip,Ilabel,rip_cont,rip_sml,tp_cont,n_link,n_runs,refuels,nr_og,start_cont):
        self.A = A
        self.n_r = n_r
        self.rows = rows
        self.cols = cols
        self.rip = rip
        self.rip_heading = np.zeros(len(rip))
        self.grids = Ilabel
        self.rip_cont = rip_cont
        self.rip_sml = rip_sml
        self.t_x = tp_cont[1]
        self.t_y = tp_cont[0]
        self.TARGET_FOUND = False
        self.n_link = n_link # Variable that links equivalent robots to original number of robots
        self.n_runs = n_runs
        self.nr_og = nr_og
        self.refuels = refuels
        self.start_cont = start_cont
        self.r_append = list()

        self.wpnts_final_list = list()
        self.dist_final_list = list()
        self.time_final_list = list()
        self.time_cumulative_list = list() # Represents cumulative time, including previous runs, take-off, landing and wait times
        self.time_totals = np.zeros([self.n_r]) # Represents flying time per run
        # self.time_totals_refuels = np.zeros([self.nr_og,self.refuels+1])
        self.dist_totals = np.zeros([self.n_r])
        self.rotations = np.zeros([self.n_r])

        # Grids: 0 is obstacle, 1 is free space
        # Graphs represent each individual node and which nodes it is connected to
        moves = np.array([[0,1],[1,0],[0,-1],[-1,0]])
        mov_class = np.array(['H','V','H','V']) # horizontal/vertical
        self.free_nodes_list = list()
        self.vertices_list = list()
        self.parents_list = list()
        self.nodes_list = list()
        self.wpnts_cont_list = list()
        self.wpnts_class_list = list()
        self.p = np.ones([self.n_r],dtype=int)*-1
        
        # Generating Paths
        for r in range(self.n_r):
            self.current_r = r
            free_nodes = np.argwhere(self.grids[r]==1) # vertice coordinates
            vertices = len(free_nodes) # number of vertices

            # Making graph for prim algorithm
            graph = np.zeros([vertices,vertices],dtype = int)
            parents = np.zeros(vertices,dtype = int)
            node_ind = 0
            for node_ind in range(vertices):
                node = free_nodes[node_ind]
                for m in range(len(moves)):
                    row = node[0]+moves[m][0]
                    col = node[1]+moves[m][1]
                    classification = mov_class[m]
                    if(row>=0)and(col>=0)and(row<self.rows)and(col<self.cols):
                        neighbour_node_ind = np.argwhere((free_nodes==(row,col)).all(axis=1))
                        if len(neighbour_node_ind)>0:
                            # Add edge to graph - weight can be different for horizontal and vertical
                            if(classification=='H'):
                                graph[node_ind][neighbour_node_ind[0][0]] = DISC_H # weights are just distance for now
                            elif(classification=='V'):
                                graph[node_ind][neighbour_node_ind[0][0]] = DISC_V # weights are just distance for now

            # JAVA MST COMMENTED OUT - indented
                # self.write_input(graph,vertices)
                # self.run_subprocess()
                # parents = self.read_output(vertices)
            parents = self.prim_algorithm(graph,vertices)

            self.free_nodes_list.append(free_nodes) # List of free mode coordinates per robot
            self.vertices_list.append(vertices)     # List of number of vertices per robot
            self.parents_list.append(parents)       # List of parent node numbers per robot

            # node object creation
            nodes = np.empty(vertices,dtype=mst_node)

            for v in range(vertices):
                nodes[v] = mst_node(v,free_nodes[v][1],free_nodes[v][0])
                nodes[v].set_edges(parents) 
            for v in range(vertices):
                self.parse_directions(nodes,nodes[v])
                nodes[v].reorganise_dir()

            self.nodes_list.append(nodes)           # List of node objects (using free nodes and parents) per robot

            # Start Arrow Creation
            self.waypoints_cont = np.zeros([vertices*4,2],dtype=float)
            self.waypoint_class = np.zeros([vertices*4],dtype=int) # 1 = Left Turn, 0 = Not bothered to classify
            self.wpnt_ind = 0
            no_arws = (vertices-1)*2 # edges = nodes-1, arrows = edges*2
            arrows = np.empty(no_arws,dtype=mst_arrow) 
            arw_ind = 0
            start_node = self.select_start_node(nodes)
            for i in range(4):
                direction = start_node.dir[i]
                node_no = start_node.edges[i]
                end_node = nodes[node_no]
                if(direction!=-1):
                    break
            arrow = mst_arrow(start_node,end_node,direction)
            self.back_turn_wpnts(arrow) # Add first two waypoints
            arrows[arw_ind] = arrow
            arw_ind += 1

            # Update Arrow and waypoint generation
            while(arw_ind!=no_arws):
                arrow = self.update_arrow(nodes,arrow.end_node,arrow.direction)
                arrows[arw_ind] = arrow
                arw_ind += 1

            # Append waypoints to larger list
            self.wpnts_cont_list.append(self.waypoints_cont)
            self.wpnts_class_list.append(self.waypoint_class)
        
        # Shifting robot initial positions by half cell - p is found during waypoint generation
        if(self.refuels > 0):
            self.head = np.zeros(len(self.rip_cont))
            # Take-off pathlengths and times
            self.TO_dist = np.zeros(len(self.rip_cont))
            self.TO_time = np.zeros(len(self.rip_cont))
            # Landing pathlengths and times
            self.LD_dist = np.zeros(len(self.rip_cont))
            self.LD_time = np.zeros(len(self.rip_cont))
        # Shift the robot starting position and caculate take off and landing times if refuelling is happening
        for r in range(self.n_r):
            if(self.refuels > 0):
                dx = self.wpnts_cont_list[r][self.p[r]][1] - self.rip[r][1] # x_shift - x_old
                dy = self.wpnts_cont_list[r][self.p[r]][0] - self.rip[r][0] # y_shift - y_old
                if(dy>0):
                    # Heading South
                    head = 270
                elif(dy<0):
                    # Heading North
                    head = 90
                elif(dx>0):
                    # Heading West
                    head = 180
                elif(dx<0):
                    # Heading East
                    head = 0
                self.head[r] = head
                ## Peddle planner - take-off
                PS = self.start_cont
                PE = self.wpnts_cont_list[r][self.p[r]]
                Head_start = 90*math.pi/180 # Start going North from landing strip
                Head_end = head*math.pi/180 # Heading at the end (rip)
                start = [PS,Head_start]
                end = [PE,Head_end]
                PP = pp.path_planner(start,end,r_min)
                PP.shortest_path()
                self.TO_dist[r] = (PP.shortest_path).PathLen # Take-off distances
                self.TO_time[r] = (PP.shortest_path).PathLen / VEL # Take-off times
                ## Pedddle planner - landing
                PS = self.wpnts_cont_list[r][self.p[r]]
                PE = self.start_cont
                Head_start = head*math.pi/180 # Start at heading it ends circuit with
                Head_end = 270*math.pi/180 # Landing going South to the landing strip
                start = [PS,Head_start]
                end = [PE,Head_end]
                PP = pp.path_planner(start,end,r_min)
                PP.shortest_path()
                self.LD_dist[r] = (PP.shortest_path).PathLen
                self.LD_time[r] = (PP.shortest_path).PathLen / VEL
                # TODO: Create schedules
                # print((PP.shortest_path).PathLen)
                # PP.plot_shortest_path('Shortest Path Landing',xaxis=2500)
                # PP.plot_paths(separate_plots=False)
            self.rip_cont[r] = self.wpnts_cont_list[r][self.p[r]]
 
        # Shifting waypoints to start at robot position and making it closed loop
        if(TARGET_FINDING):
            print("Robot:",self.TARGET_CELL[0],"Ind: ",self.TARGET_CELL[1],"Waypoint: ",self.wpnts_cont_list[self.TARGET_CELL[0]][self.TARGET_CELL[1]],"Target Location: ",self.t_y,self.t_x)
        self.update_wpnts()
   
    def update_wpnts(self):
        for r in range(self.n_r):
            wpnts = self.wpnts_cont_list[r]
            wpnts_class = self.wpnts_class_list[r]
            wpnts_updated = np.zeros([len(wpnts)+1,2],dtype=float)
            wpnts_class_updated = np.zeros([len(wpnts_class)+1],dtype=float)
            p = self.p[r]
            ind = 0
            ind_p = p
            KEEP_SEARCHING = True
            while(ind_p!=len(wpnts)):
                wpnts_updated[ind] = wpnts[ind_p]
                wpnts_class_updated[ind] = wpnts_class[ind_p]
                if(TARGET_FINDING==True)and(KEEP_SEARCHING):
                    if(self.TARGET_CELL[1]==ind_p)and(self.TARGET_CELL[0]==r):
                        self.TARGET_CELL[1]=ind
                        KEEP_SEARCHING = False
                ind_p+=1
                ind+=1
            ind_p=0
            while(ind_p!=p):
                wpnts_updated[ind] = wpnts[ind_p]
                wpnts_class_updated[ind] = wpnts_class[ind_p]
                if(TARGET_FINDING==True)and(KEEP_SEARCHING):
                    if(self.TARGET_CELL[1]==ind_p)and(self.TARGET_CELL[0]==r):
                        self.TARGET_CELL[1]=ind
                        KEEP_SEARCHING = False
                ind_p+=1
                ind+=1  
            wpnts_updated[ind] = wpnts[p]
            wpnts_class_updated[ind] = wpnts_class[p]
            self.wpnts_cont_list[r] = wpnts_updated
            self.wpnts_class_list[r] = wpnts_class_updated

    def select_start_node(self,nodes):
        for node in nodes:
            # Select first node with only one edge
            if(node.no_edges==1):
                return(node)

    def parse_directions(self,nodes,node):
        start_x = node.coord_x
        start_y = node.coord_y
        for i in range(len(node.edges)):
            edge = node.edges[i]
            new_node = nodes[edge]
            if(edge!=-1):
                end_x = new_node.coord_x # col
                end_y = new_node.coord_y # row
                dx = end_x - start_x
                dy = end_y - start_y
                if(dy==1):
                    node.dir[i]= 2 # South
                if(dy==-1):
                    node.dir[i]= 0 # North
                if(dx==1):
                    node.dir[i]= 1 # East
                if(dx==-1):
                    node.dir[i]= 3 # West

    def update_arrow(self,nodes,start_node,prev_dir):
        edges = start_node.edges
        direction = start_node.dir

        P = np.array([[3,0,1,2],[0,1,2,3],[1,2,3,0],[2,3,0,1]]) # Stores L,F,R,B for each prev_dir -> N,E,S,W ~ 0,1,2,3 

        p = P[prev_dir] # L,F,R,B
        # Left
        if(direction[p[0]]!=-1):
            ind = p[0]
            new_dir = 'L'
        # Forward
        elif(direction[p[1]]!=-1):
            ind = p[1]
            new_dir = 'F'
        # Right
        elif(direction[p[2]]!=-1):
            ind = p[2]
            new_dir = 'R'
        # Backttracking
        elif(direction[p[3]]!=-1):
            ind = p[3]
            new_dir = 'B'
        node_ind = edges[ind]
        end_node = nodes[node_ind]
        direction = direction[ind]
        arrow = mst_arrow(start_node,end_node,direction)

        # Waypoint Generation
        if(new_dir=='L'):
            self.left_turn_wpnts(arrow)
        elif(new_dir=='F'):
            self.fw_turn_wpnts(arrow)
        elif(new_dir=='R'):
            self.right_turn_wpnts(arrow)
        elif(new_dir=='B'):
            self.back_turn_wpnts(arrow)
        return(arrow)   

    def left_turn_wpnts(self,arrow):
        # Add one waypoint - already done half CCW shift (comments are mostly here)
        direction = arrow.direction
        end_node = arrow.end_node
        X = end_node.coord_x
        Y = end_node.coord_y
        if(direction==0):
            # This means it went from E-N
            # OVERALL - the point is half shifted downwards on the plane (South shift)
            x = 2*X # original wpnt location - sml grid
            y_c = 2*Y + 1 # original wpnt location - sml grid
            y = y_c + 0.5 # y half shift on sml grid - DOWN shift
            
            x = (x + 0.5)*DISC_H # continuous waypoint (half shifted - although in this case the x value is unchanged after the half shift)
            x_c = x # continuous waypoint (unshifted - centre of small cell)
            y = (self.rows*2 - (y + 0.5))*DISC_V # continuous waypoint (half shifted)
            y_c = (self.rows*2 - (y_c + 0.5))*DISC_V # continuous waypoint (unshifted)
        elif(direction==1):
            # This means it went from S-E
            x_c = 2*X
            x = x_c - 0.5
            y = 2*Y

            x = (x + 0.5)*DISC_H
            x_c = (x_c + 0.5)*DISC_H
            y = (self.rows*2 - y - 0.5)*DISC_V
            y_c = y
        elif(direction==2):
            # This means it went from W-S
            x = 2*X + 1
            y_c = 2*Y
            y = y_c - 0.5
            
            x = (x + 0.5)*DISC_H
            x_c = x
            y = (self.rows*2 - y - 0.5)*DISC_V
            y_c = (self.rows*2 - y_c - 0.5)*DISC_V
        elif(direction==3):        
            # This means it went from N-W
            x_c = 2*X + 1
            x = x_c + 0.5
            y = 2*Y + 1

            x = (x + 0.5)*DISC_H
            x_c = (x_c + 0.5)*DISC_H
            y = (self.rows*2 - y - 0.5)*DISC_V
            y_c = y           
        #  Code that checks if robot starting position is at this waypoint
        if(math.isclose(self.rip_cont[self.current_r][0],y_c))and(math.isclose(self.rip_cont[self.current_r][1],x_c)):
            self.p[self.current_r] = self.wpnt_ind
        self.waypoints_cont[self.wpnt_ind][0] = y
        self.waypoints_cont[self.wpnt_ind][1] = x
        self.waypoint_class[self.wpnt_ind] = 1
        self.wpnt_ind+=1
        if(TARGET_FINDING==True):
            if(self.TARGET_FOUND == False):
                if(self.t_x<=x_c+DISC_H/2)and(self.t_x>=x_c-DISC_H/2)and(self.t_y<=y_c+DISC_V/2)and(self.t_y>=y_c-DISC_V/2):
                    # Target is within this cell
                    self.TARGET_CELL = np.array([self.current_r,self.wpnt_ind-1]) # stores robot and index at which target is
                    self.TARGET_FOUND = True
    
    def fw_turn_wpnts(self,arrow):
        # Add two waypoints
        direction = arrow.direction
        end_node = arrow.end_node
        X = end_node.coord_x
        Y = end_node.coord_y
        if(direction==0):
            # This means it went from N-N
            x1 = 2*X
            y1_c = 2*Y + 2
            y1 = y1_c + 0.5
            
            x2 = 2*X
            y2_c = 2*Y + 1
            y2 = y2_c + 0.5
            
            # Change to continuous
            y1 = (self.rows*2 - y1 - 0.5)*DISC_V
            y1_c = (self.rows*2 - y1_c - 0.5)*DISC_V
            x1 = (x1 + 0.5)*DISC_H
            x1_c = x1

            y2 = (self.rows*2 - y2 - 0.5)*DISC_V
            y2_c = (self.rows*2 - y2_c - 0.5)*DISC_V
            x2 = (x2 + 0.5)*DISC_H
            x2_c = x2
        elif(direction==1):
            # This means it went from E-E
            x1_c = 2*X - 1
            x1 = x1_c - 0.5
            y1 = 2*Y

            x2_c = 2*X
            x2 = x2_c - 0.5
            y2 = 2*Y
            # Change to continuous
            y1 = (self.rows*2 - y1 - 0.5)*DISC_V
            y1_c = y1
            x1 = (x1 + 0.5)*DISC_H
            x1_c = (x1_c + 0.5)*DISC_H

            y2 = (self.rows*2 - y2 - 0.5)*DISC_V
            y2_c = y2
            x2 = (x2 + 0.5)*DISC_H
            x2_c = (x2_c + 0.5)*DISC_H
        elif(direction==2):
            # This means it went from S-S
            x1 = 2*X + 1
            y1_c = 2*Y - 1
            y1 = y1_c - 0.5

            x2 = 2*X + 1
            y2_c = 2*Y
            y2 = y2_c - 0.5
            
            # Change to continuous
            y1 = (self.rows*2 - y1 - 0.5)*DISC_V
            y1_c = (self.rows*2 - y1_c - 0.5)*DISC_V
            x1 = (x1 + 0.5)*DISC_H
            x1_c = x1

            y2 = (self.rows*2 - y2 - 0.5)*DISC_V
            y2_c = (self.rows*2 - y2_c - 0.5)*DISC_V
            x2 = (x2 + 0.5)*DISC_H
            x2_c = x2
        elif(direction==3):        
            # This means it went from W-W
            x1_c = 2*X + 2
            x1 = x1_c + 0.5
            y1 = 2*Y + 1

            x2_c = 2*X + 1
            x2 = x2_c + 0.5
            y2 = 2*Y + 1

            # Change to continuous
            y1 = (self.rows*2 - y1 - 0.5)*DISC_V
            y1_c = y1
            x1 = (x1 + 0.5)*DISC_H
            x1_c = (x1_c + 0.5)*DISC_H

            y2 = (self.rows*2 - y2 - 0.5)*DISC_V
            y2_c = y2
            x2 = (x2 + 0.5)*DISC_H
            x2_c = (x2_c + 0.5)*DISC_H
        if(math.isclose(self.rip_cont[self.current_r][0],y1_c))and(math.isclose(self.rip_cont[self.current_r][1],x1_c)):
            self.p[self.current_r] = self.wpnt_ind
        self.waypoints_cont[self.wpnt_ind][0] = y1
        self.waypoints_cont[self.wpnt_ind][1] = x1
        self.wpnt_ind+=1
        if(math.isclose(self.rip_cont[self.current_r][0],y2_c))and(math.isclose(self.rip_cont[self.current_r][1],x2_c)):
            self.p[self.current_r] = self.wpnt_ind
        self.waypoints_cont[self.wpnt_ind][0] = y2
        self.waypoints_cont[self.wpnt_ind][1] = x2
        self.wpnt_ind+=1
        if(TARGET_FINDING==True):
            if(self.TARGET_FOUND == False):
                if(self.t_x<=x1_c+DISC_H/2)and(self.t_x>=x1_c-DISC_H/2)and(self.t_y<=y1_c+DISC_V/2)and(self.t_y>=y1_c-DISC_V/2):
                    # Target is within this cell
                    self.TARGET_CELL = np.array([self.current_r,self.wpnt_ind-2]) # stores robot and index at which target is
                    self.TARGET_FOUND = True
                elif(self.t_x<=x2_c+DISC_H/2)and(self.t_x>=x2_c-DISC_H/2)and(self.t_y<=y2_c+DISC_V/2)and(self.t_y>=y2_c-DISC_V/2):
                    self.TARGET_CELL = np.array([self.current_r,self.wpnt_ind-1])
                    self.TARGET_FOUND = True
    
    def right_turn_wpnts(self,arrow):
        # Add three waypoints
        direction = arrow.direction
        end_node = arrow.end_node
        X = end_node.coord_x
        Y = end_node.coord_y
        if(direction==0):
            # This means it went from W-N
            x1_c = 2*X
            x1 = x1_c + 0.5
            y1 = 2*Y + 3

            x2 = 2*X 
            y2_c = 2*Y + 2
            y2 = y2_c + 0.5

            x3 = 2*X
            y3_c = 2*Y + 1
            y3 = y3_c + 0.5

            # Change to continuous
            y1 = (self.rows*2 - y1 - 0.5)*DISC_V
            y1_c = y1
            x1 = (x1 + 0.5)*DISC_H
            x1_c = (x1_c + 0.5)*DISC_H
            y2 = (self.rows*2 - y2 - 0.5)*DISC_V
            y2_c = (self.rows*2 - y2_c - 0.5)*DISC_V
            x2 = (x2 + 0.5)*DISC_H
            x2_c = x2
            y3 = (self.rows*2 - y3 - 0.5)*DISC_V
            y3_c = (self.rows*2 - y3_c - 0.5)*DISC_V
            x3 = (x3 + 0.5)*DISC_H
            x3_c = x3
        elif(direction==1):
            # This means it went from N-E
            x1 = 2*X - 2
            y1_c = 2*Y
            y1 = y1_c + 0.5

            x2_c = 2*X - 1
            x2 = x2_c - 0.5
            y2 = 2*Y

            x3_c = 2*X
            x3 = x3_c - 0.5
            y3 = 2*Y

            # Change to continuous
            y1 = (self.rows*2 - y1 - 0.5)*DISC_V
            y1_c = (self.rows*2 - y1_c - 0.5)*DISC_V
            x1 = (x1 + 0.5)*DISC_H
            x1_c = x1

            y2 = (self.rows*2 - y2 - 0.5)*DISC_V
            y2_c = y2
            x2 = (x2 + 0.5)*DISC_H
            x2_c = (x2_c + 0.5)*DISC_H

            y3 = (self.rows*2 - y3 - 0.5)*DISC_V
            y3_c = y3
            x3 = (x3 + 0.5)*DISC_H
            x3_c = (x3_c + 0.5)*DISC_H
        elif(direction==2):
            # This means it went from E-S
            x1_c = 2*X + 1
            x1 = x1_c - 0.5
            y1 = 2*Y - 2

            x2 = 2*X + 1
            y2_c = 2*Y - 1 
            y2 = y2_c - 0.5

            x3 = 2*X + 1
            y3_c = 2*Y
            y3 = y3_c - 0.5

            # Change to continuous
            y1 = (self.rows*2 - y1 - 0.5)*DISC_V
            y1_c = y1
            x1 = (x1 + 0.5)*DISC_H
            x1_c = (x1_c + 0.5)*DISC_H

            y2 = (self.rows*2 - y2 - 0.5)*DISC_V
            y2_c = (self.rows*2 - y2_c - 0.5)*DISC_V
            x2 = (x2 + 0.5)*DISC_H
            x2_c = x2

            y3 = (self.rows*2 - y3 - 0.5)*DISC_V
            y3_c = (self.rows*2 - y3_c - 0.5)*DISC_V
            x3 = (x3 + 0.5)*DISC_H
            x3_c = x3
        elif(direction==3):        
            # This means it went from S-W
            x1 = 2*X + 3
            y1_c = 2*Y + 1
            y1 = y1_c - 0.5

            x2_c = 2*X + 2
            x2 = x2_c + 0.5
            y2 = 2*Y + 1

            x3_c = 2*X + 1
            x3 = x3_c + 0.5
            y3 = 2*Y + 1

            # Change to continuous
            y1 = (self.rows*2 - y1 - 0.5)*DISC_V
            y1_c = (self.rows*2 - y1_c - 0.5)*DISC_V
            x1 = (x1 + 0.5)*DISC_H
            x1_c = x1

            y2 = (self.rows*2 - y2 - 0.5)*DISC_V
            y2_c = y2
            x2 = (x2 + 0.5)*DISC_H
            x2_c = (x2_c + 0.5)*DISC_H

            y3 = (self.rows*2 - y3 - 0.5)*DISC_V
            y3_c = y3
            x3 = (x3 + 0.5)*DISC_H
            x3_c = (x3_c + 0.5)*DISC_H
        if(math.isclose(self.rip_cont[self.current_r][0],y1_c))and(math.isclose(self.rip_cont[self.current_r][1],x1_c)):
            self.p[self.current_r] = self.wpnt_ind
        self.waypoints_cont[self.wpnt_ind][0] = y1
        self.waypoints_cont[self.wpnt_ind][1] = x1
        self.wpnt_ind+=1
        if(math.isclose(self.rip_cont[self.current_r][0],y2_c))and(math.isclose(self.rip_cont[self.current_r][1],x2_c)):
            self.p[self.current_r] = self.wpnt_ind
        self.waypoints_cont[self.wpnt_ind][0] = y2
        self.waypoints_cont[self.wpnt_ind][1] = x2
        self.wpnt_ind+=1
        if(math.isclose(self.rip_cont[self.current_r][0],y3_c))and(math.isclose(self.rip_cont[self.current_r][1],x3_c)):
            self.p[self.current_r] = self.wpnt_ind
        self.waypoints_cont[self.wpnt_ind][0] = y3
        self.waypoints_cont[self.wpnt_ind][1] = x3
        self.wpnt_ind+=1
        if(TARGET_FINDING==True):
            if(self.TARGET_FOUND == False):
                if(self.t_x<=x1_c+DISC_H/2)and(self.t_x>=x1_c-DISC_H/2)and(self.t_y<=y1_c+DISC_V/2)and(self.t_y>=y1_c-DISC_V/2):
                    # Target is within this cell
                    self.TARGET_CELL = np.array([self.current_r,self.wpnt_ind-3]) # stores robot and index at which target is
                    self.TARGET_FOUND = True
                elif(self.t_x<=x2_c+DISC_H/2)and(self.t_x>=x2_c-DISC_H/2)and(self.t_y<=y2_c+DISC_V/2)and(self.t_y>=y2_c-DISC_V/2):
                    self.TARGET_CELL = np.array([self.current_r,self.wpnt_ind-2])
                    self.TARGET_FOUND = True
                elif(self.t_x<=x3_c+DISC_H/2)and(self.t_x>=x3_c-DISC_H/2)and(self.t_y<=y3_c+DISC_V/2)and(self.t_y>=y3_c-DISC_V/2):
                    self.TARGET_CELL = np.array([self.current_r,self.wpnt_ind-1])
                    self.TARGET_FOUND = True
    
    def back_turn_wpnts(self,arrow):
        # Add four waypoints
        direction = arrow.direction
        end_node = arrow.end_node
        X = end_node.coord_x
        Y = end_node.coord_y
        if(direction==0):
            # This means it went from S-N
            x1 = 2*X + 1
            y1_c = 2*Y + 3
            y1 = y1_c - 0.5

            x2_c = 2*X
            x2 = x2_c + 0.5
            y2 = 2*Y + 3

            x3 = 2*X
            y3_c = 2*Y + 2
            y3 = y3_c + 0.5

            x4 = 2*X
            y4_c = 2*Y + 1
            y4 = y4_c + 0.5

            # adjust to continuous values
            x1 = (x1 + 0.5)*DISC_H
            x1_c = x1
            y1 = (self.rows*2 - y1 - 0.5)*DISC_V
            y1_c = (self.rows*2 - y1_c - 0.5)*DISC_V

            x2 = (x2 + 0.5)*DISC_H
            x2_c = (x2_c + 0.5)*DISC_H
            y2 = (self.rows*2 - y2 - 0.5)*DISC_V
            y2_c = y2

            x3 = (x3 + 0.5)*DISC_H
            x3_c = x3 
            y3 = (self.rows*2 - y3 - 0.5)*DISC_V
            y3_c = (self.rows*2 - y3_c - 0.5)*DISC_V

            x4 = (x4 + 0.5)*DISC_H
            x4_c = x4
            y4 = (self.rows*2 - y4 - 0.5)*DISC_V
            y4_c = (self.rows*2 - y4_c - 0.5)*DISC_V      
        elif(direction==1):
            # This means it went from W-E
            x1_c = 2*X - 2
            x1 = x1_c + 0.5
            y1 = 2*Y + 1

            x2 = 2*X - 2
            y2_c = 2*Y
            y2 = y2_c + 0.5

            x3_c = 2*X - 1
            x3 = x3_c - 0.5
            y3 = 2*Y

            x4_c = 2*X
            x4 = x4_c - 0.5
            y4 = 2*Y

            # adjust to continuous values
            x1 = (x1 + 0.5)*DISC_H
            x1_c = (x1_c + 0.5)*DISC_H
            y1 = (self.rows*2 - y1 - 0.5)*DISC_V
            y1_c = y1

            x2 = (x2 + 0.5)*DISC_H
            x2_c = x2
            y2 = (self.rows*2 - y2 - 0.5)*DISC_V
            y2_c = (self.rows*2 - y2_c - 0.5)*DISC_V

            x3 = (x3 + 0.5)*DISC_H
            x3_c = (x3_c + 0.5)*DISC_H
            y3 = (self.rows*2 - y3 - 0.5)*DISC_V
            y3_c = y3

            x4 = (x4 + 0.5)*DISC_H
            x4_c = (x4_c + 0.5)*DISC_H
            y4 = (self.rows*2 - y4 - 0.5)*DISC_V
            y4_c = y4           
        elif(direction==2):
            # This means it went from N-S
            x1 = 2*X
            y1_c = 2*Y - 2
            y1 = y1_c + 0.5

            x2_c = 2*X + 1
            x2 = x2_c - 0.5
            y2 = 2*Y - 2

            x3 = 2*X + 1
            y3_c = 2*Y - 1
            y3 = y3_c - 0.5

            x4 = 2*X + 1
            y4_c = 2*Y
            y4 = y4_c - 0.5

            # adjust to continuous values
            x1 = (x1 + 0.5)*DISC_H
            x1_c = x1
            y1 = (self.rows*2 - y1 - 0.5)*DISC_V
            y1_c = (self.rows*2 - y1_c - 0.5)*DISC_V
            
            x2 = (x2 + 0.5)*DISC_H
            x2_c = (x2_c + 0.5)*DISC_H 
            y2 = (self.rows*2 - y2 - 0.5)*DISC_V
            y2_c = y2
            
            x3 = (x3 + 0.5)*DISC_H
            x3_c = x3
            y3 = (self.rows*2 - y3 - 0.5)*DISC_V
            y3_c = (self.rows*2 - y3_c - 0.5)*DISC_V 
            
            x4 = (x4 + 0.5)*DISC_H
            x4_c = x4
            y4 = (self.rows*2 - y4 - 0.5)*DISC_V
            y4_c = (self.rows*2 - y4_c - 0.5)*DISC_V         
        elif(direction==3):        
            # This means it went from E-W
            x1_c = 2*X + 3
            x1 = x1_c - 0.5
            y1 = 2*Y

            x2 = 2*X + 3
            y2_c = 2*Y + 1
            y2 = y2_c - 0.5

            x3_c = 2*X + 2
            x3 = x3_c + 0.5
            y3 = 2*Y + 1

            x4_c = 2*X + 1
            x4 = x4_c + 0.5
            y4 = 2*Y + 1

            # adjust to continuous values
            x1 = (x1 + 0.5)*DISC_H
            x1_c = (x1_c + 0.5)*DISC_H
            y1 = (self.rows*2 - y1 - 0.5)*DISC_V
            y1_c = y1

            x2 = (x2 + 0.5)*DISC_H
            x2_c = x2
            y2 = (self.rows*2 - y2 - 0.5)*DISC_V
            y2_c = (self.rows*2 - y2_c - 0.5)*DISC_V

            x3 = (x3 + 0.5)*DISC_H
            x3_c = (x3_c + 0.5)*DISC_H
            y3 = (self.rows*2 - y3 - 0.5)*DISC_V
            y3_c = y3

            x4 = (x4 + 0.5)*DISC_H
            x4_c = (x4_c + 0.5)*DISC_H
            y4 = (self.rows*2 - y4 - 0.5)*DISC_V
            y4_c = y4           
        if(math.isclose(self.rip_cont[self.current_r][0],y1_c))and(math.isclose(self.rip_cont[self.current_r][1],x1_c)):
            self.p[self.current_r] = self.wpnt_ind
        self.waypoints_cont[self.wpnt_ind][0] = y1
        self.waypoints_cont[self.wpnt_ind][1] = x1
        self.wpnt_ind+=1
        if(math.isclose(self.rip_cont[self.current_r][0],y2_c))and(math.isclose(self.rip_cont[self.current_r][1],x2_c)):
            self.p[self.current_r] = self.wpnt_ind
        self.waypoints_cont[self.wpnt_ind][0] = y2
        self.waypoints_cont[self.wpnt_ind][1] = x2
        self.wpnt_ind+=1
        if(math.isclose(self.rip_cont[self.current_r][0],y3_c))and(math.isclose(self.rip_cont[self.current_r][1],x3_c)):
            self.p[self.current_r] = self.wpnt_ind
        self.waypoints_cont[self.wpnt_ind][0] = y3
        self.waypoints_cont[self.wpnt_ind][1] = x3
        self.wpnt_ind+=1
        if(math.isclose(self.rip_cont[self.current_r][0],y4_c))and(math.isclose(self.rip_cont[self.current_r][1],x4_c)):
            self.p[self.current_r] = self.wpnt_ind
        self.waypoints_cont[self.wpnt_ind][0] = y4
        self.waypoints_cont[self.wpnt_ind][1] = x4
        self.wpnt_ind+=1
        if(TARGET_FINDING==True):
            if(self.TARGET_FOUND == False):
                if(self.t_x<=x1_c+DISC_H/2)and(self.t_x>=x1_c-DISC_H/2)and(self.t_y<=y1_c+DISC_V/2)and(self.t_y>=y1_c-DISC_V/2):
                    # Target is within this cell
                    self.TARGET_CELL = np.array([self.current_r,self.wpnt_ind-4]) # stores robot and index at which target is
                    self.TARGET_FOUND = True
                elif(self.t_x<=x2_c+DISC_H/2)and(self.t_x>=x2_c-DISC_H/2)and(self.t_y<=y2_c+DISC_V/2)and(self.t_y>=y2_c-DISC_V/2):
                    self.TARGET_CELL = np.array([self.current_r,self.wpnt_ind-3])
                    self.TARGET_FOUND = True
                elif(self.t_x<=x3_c+DISC_H/2)and(self.t_x>=x3_c-DISC_H/2)and(self.t_y<=y3_c+DISC_V/2)and(self.t_y>=y3_c-DISC_V/2):
                    self.TARGET_CELL = np.array([self.current_r,self.wpnt_ind-2])
                    self.TARGET_FOUND = True
                elif(self.t_x<=x4_c+DISC_H/2)and(self.t_x>=x4_c-DISC_H/2)and(self.t_y<=y4_c+DISC_V/2)and(self.t_y>=y4_c-DISC_V/2):
                    self.TARGET_CELL = np.array([self.current_r,self.wpnt_ind-1])
                    self.TARGET_FOUND = True

    def waypoint_final_generation(self,wpnts,wpnts_class,r,start_timestamp):
        # Waypoint, distance and time arrays created to describe the path
        # Includes graph drawing for tree and paths
        rotations = 0
        wpnts_final = list()
        dist_final = list() # NON-Cumulative
        # Append first waypoint
        wpnts_final.append([wpnts[0][1],wpnts[0][0]])
        dist_final.append(0) # Distance linked to first waypoint is 0
        # Initialise totals to 0
        time_break = False
        dist_tot = 0
        time_tot = 0
        r_og = self.n_link[r] # Original robot
        refuel_run = self.n_runs[r] # Which refuel run the robot is on

        # Initialise time_tot to last time on the previous run - further offsets can only be added after the path lengths are known
        # if(refuel_run == 0):
        #     start_timestamp = 0
        # elif(refuel_run > 0):
        #     # The last element of the previous equivalent robot's time array would represent the total time taken by the same robot in the previous run
        #     start_timestamp = self.time_cumulative_list[r-1][-1] 
        for w in range(len(wpnts)-1):
            x1 = wpnts[w][1]
            x2 = wpnts[w+1][1]
            y1 = wpnts[w][0]
            y2 = wpnts[w+1][0]
            
            # Generate the appropriate waypoints 
            if(x2>x1):
                if(y2>y1):
                    # F U
                    rotations += 1 
                    if(wpnts_class[w+1]==1):
                    # Bottom - Left Turn (Note: Logic table says that only left turns are different, therefore they are detected using wpnts_class)
                        ## Long leg (Line waypoint)
                        wpnts_final.append([x2-r_min,y1]) # End of long line
                        l1 = DISC_H/2 - r_min # length of long leg
                        dist_final.append(l1)
                        ## Circular waypoint
                        wpnts_final.append([x2-r_min,y1+r_min,2*r_min,270.0]) # Arc
                        l2 = r_min*np.pi/2 # arc length
                        dist_final.append(l2)
                        ## Short leg (Line waypoint)
                        if(r_min != r_max): # Radius is smaller than size of square
                            # This point is equivalent to [x2,y2], which marks the end of an arc, when r_min == r_max
                            wpnts_final.append([x2,y1+r_min]) # Start of short line
                            dist_final.append(0) # Distance hasn't increased since end of arc  
                        wpnts_final.append([x2,y2]) # End of short line
                        l3 = DISC_V/2 - r_min # length of short leg (same as long leg for square discretisation)
                        dist_final.append(l3)
                        dist_tot = dist_tot + l1 + l2 + l3                           
                    else:
                    # Top - Backtrack / Right
                        ## Short line
                        if(r_min != r_max):
                            wpnts_final.append([x1,y2-r_min]) # End of short line (arc start)
                            l1 = DISC_V/2 - r_min # short leg length
                            dist_final.append(l1)
                        ## Circular Waypoint
                        wpnts_final.append([x1+r_min,y2-r_min,2*r_min,90.0]) # Arc
                        l2 = r_min*np.pi/2 # arc length
                        dist_final.append(l2)
                        ## Long line
                        wpnts_final.append([x1+r_min,y2]) # Start of long line (arc end)
                        dist_final.append(0)
                        wpnts_final.append([x2,y2]) # End of long line
                        l3 = DISC_H/2 - r_min # long leg   length                      
                        dist_final.append(l3)
                        dist_tot = dist_tot + l1 + l2 + l3
                elif(y2<y1):
                    # B D
                    rotations += 1 
                    if(wpnts_class[w+1]==1):
                    # Bottom - Left Turn
                        ## Short line
                        if(r_min != r_max):
                            wpnts_final.append([x1,y2+r_min])
                            l1 = DISC_V/2 - r_min # short line length
                            dist_final.append(l1)
                        ## Circular waypoint
                        wpnts_final.append([x1+r_min,y2+r_min,2*r_min,180.0])
                        l2 = r_min*np.pi/2 # arc length
                        dist_final.append(l2)
                        ## Long line
                        wpnts_final.append([x1+r_min,y2]) # Start of long line
                        dist_final.append(0)
                        wpnts_final.append([x2,y2]) # End of long line
                        l3 = DISC_H/2 - r_min # long line length
                        dist_final.append(l3)
                        dist_tot = dist_tot + l1 + l2 + l3              
                    else:
                    # Top - Right Turn / Backtrack
                        ## Long line
                        wpnts_final.append([x2-r_min,y1]) # End of long line
                        l1 = DISC_H/2 - r_min # long line length
                        dist_final.append(l1)
                        ## Circular waypoint
                        wpnts_final.append([x2-r_min,y1-r_min,2*r_min,0.0]) # arc
                        l2 = r_min*np.pi/2 # arc length
                        dist_final.append(l2)
                        ## Short line
                        if(r_min != r_max):
                            wpnts_final.append([x2,y1-r_min]) # Start of short line
                            dist_final.append(0)
                        wpnts_final.append([x2,y2]) # End of short line
                        l3 = DISC_V/2 - r_min # short line length
                        dist_final.append(l3)
                        dist_tot = dist_tot + l1 + l2 + l3
                else: # y2 == y1
                # Vertical line
                    wpnts_final.append([x2,y2]) # End of vertical line
                    dist_final.append(DISC_V)
                    dist_tot = dist_tot + DISC_V
            elif(x2<x1):
                if(y2>y1):
                    # B U
                    rotations += 1 
                    if(wpnts_class[w+1]==1):
                    # Top - Left Turn
                        ## Short line
                        if(r_min != r_max):
                            wpnts_final.append([x1,y2-r_min]) # End of short line
                            l1 = DISC_V/2 - r_min # Short line length
                            dist_final.append(l1)
                        ## Circular waypoint
                        wpnts_final.append([x1-r_min,y2-r_min,2*r_min,0.0])
                        l2 = r_min*np.pi/2 # arc length
                        dist_final.append(l2)
                        ## Long length
                        wpnts_final.append([x1-r_min,y2]) # Start of long line
                        dist_final.append(0)
                        wpnts_final.append([x2,y2]) # End of long line                       
                        l3 = DISC_H/2 - r_min # long line length
                        dist_final.append(l3)
                        dist_tot = dist_tot + l1 + l2 + l3
                    else:
                    # Bottom - Right Turn / Backtrack
                        ## Long line
                        wpnts_final.append([x2+r_min,y1]) # End of long line
                        l1 = DISC_H/2 - r_min # long line length
                        dist_final.append(l1)
                        ## Circular waypoint
                        wpnts_final.append([x2+r_min,y1+r_min,2*r_min,180.0]) # arc
                        l2 = r_min*np.pi/2 # arc length
                        dist_final.append(l2)
                        ## Short line
                        if(r_min != r_max):
                            wpnts_final.append([x2,y1+r_min]) # Start of short line
                            dist_final.append(0)
                        wpnts_final.append([x2,y2]) # End of short line                    
                        l3 = DISC_V/2 - r_min # short line length
                        dist_final.append(l3)
                        dist_tot = dist_tot + l1 + l2 + l3
                elif(y2<y1):
                    # F D
                    rotations += 1 
                    if(wpnts_class[w+1]==1):
                    # Top - Left
                        ## Long line
                        wpnts_final.append([x2+r_min,y1]) # End of long line
                        l1 = DISC_H/2 - r_min # long line length
                        dist_final.append(l1)
                        ## Circular waypoint
                        wpnts_final.append([x2+r_min,y1-r_min,2*r_min,90.0]) # Arc
                        l2 = r_min*np.pi/2 # arc length
                        dist_final.append(l2)
                        ## Short line
                        if(r_min != r_max):
                            wpnts_final.append([x2,y1-r_min]) # Start of short line
                            dist_final.append(0)
                        wpnts_final.append([x2,y2]) # End of short line
                        l3 = DISC_V/2 - r_min # short line length
                        dist_final.append(l3)
                        dist_tot = dist_tot + l1 + l2 + l3
                    else:
                    # Bottom - Right Turn / Backtrack      
                        ## Short line
                        if(r_min != r_max):
                            wpnts_final.append([x1,y2+r_min]) # short line end
                            l1 = DISC_V/2 - r_min # short line length
                            dist_final.append(l1)
                        ## Circular waypoint
                        wpnts_final.append([x1-r_min,y2+r_min,2*r_min,270.0]) # arc
                        l2 = r_min*np.pi/2 # arc length
                        dist_final.append(l2)
                        ## Long line
                        wpnts_final.append([x1-r_min,y2]) # long line start
                        dist_final.append(0)
                        wpnts_final.append([x2,y2]) # long line end
                        l3 = DISC_H/2 - r_min # long line length
                        dist_final.append(l3)
                        dist_tot = dist_tot + l1 + l2 + l3 
                else: # y2 == y1
                # Vertical line
                    wpnts_final.append([x2,y2]) # End of line
                    dist_final.append(DISC_V)
                    dist_tot = dist_tot + DISC_V
            else: # x2 == x1
            # Horizontal line
                wpnts_final.append([x2,y2]) # End of horizontal line
                dist_final.append(DISC_H)
                dist_tot = dist_tot + DISC_H
            if(TARGET_FINDING):
                if(r==self.TARGET_CELL[0]):
                    # If it's the robot that finds the goal
                    if(w==self.TARGET_CELL[1]):
                        self.DISTANCE_BREAK = dist_tot

        # Distance
        self.wpnts_final_list.append(wpnts_final) # Waypoints array
        self.dist_final_list.append(dist_final) # Distance array
        self.dist_totals[r] = dist_tot # Total Distance for robot r
        dist_tot = 0
        # Time
        time_final = np.zeros(len(dist_final))
        time_cumulative = np.zeros(len(dist_final))
        for d in range(len(dist_final)):
            time_final[d] = dist_final[d] / VEL
            time_tot = time_tot + time_final[d] # start_timestamp represents offset as a result of refuelling
            time_cumulative[d] = start_timestamp + time_tot
            if((TARGET_FINDING)and(r==self.TARGET_CELL[0])and(time_break==False)):
                dist_tot = dist_tot + dist_final[d]
                if(dist_tot>=self.DISTANCE_BREAK):
                    self.TIME_BREAK = time_cumulative[d]
                    time_break = True
        self.time_final_list.append(time_final)
        self.time_cumulative_list.append(time_cumulative)
        self.r_append.append(r)
        self.time_totals[r] = time_tot # Total Time for equivalent robot r to complete path
        # Rotations
        self.rotations[r] = rotations

    def print_graph(self,nodes,parents,ax,r,time_start=0,time_end=np.Inf):
        # Graph printing
        # Plot spanning tree
        if(PRINT_TREE==True):
            for node in nodes:
                x = ( (node[1])*2 +1 )*DISC_H
                y = ( (self.rows - node[0] - 1)*2 + 1 )*DISC_V
                plt.plot(x,y,".",markersize=S_MARKERIZE,color=TREE_COLOR)
            for i in range(1,len(parents)):
                x0 = ( (nodes[i][1])*2 + 1 )*DISC_H
                x1 = ( (nodes[parents[i]][1])*2 + 1 )*DISC_H
                y0 = ( (self.rows - nodes[i][0] - 1)*2 + 1 )*DISC_V
                y1 = ( (self.rows - nodes[parents[i]][0] - 1)*2 + 1 )*DISC_V
                plt.plot(np.array([x0,x1]),np.array([y0,y1]),"-",linewidth=LINEWIDTH,color=TREE_COLOR)
        # Plot connected waypoints
        if(PRINT_PATH==True):
            # The distance and time arrays are the same length as the waypoints array
            circ_flag = 0
            for w in range(1,len(self.wpnts_final_list[r])):
                # dist = self.dist_final_list[r][w]
                time = self.time_cumulative_list[r][w]
                if(time >= time_start)and(time <= time_end):
                    if(len(self.wpnts_final_list[r][w])==2): # line
                        if(circ_flag == 0):    
                            plt.plot([self.wpnts_final_list[r][w-1][0],self.wpnts_final_list[r][w][0]],[self.wpnts_final_list[r][w-1][1],self.wpnts_final_list[r][w][1]],'-',linewidth=LINEWIDTH,color=PATH_COLOR)
                            plt.plot([self.wpnts_final_list[r][w-1][0],self.wpnts_final_list[r][w][0]],[self.wpnts_final_list[r][w-1][1],self.wpnts_final_list[r][w][1]],'.',markersize=S_MARKERIZE,color=PATH_COLOR)
                        else:
                            circ_flag = 0
                    else: # circle
                        if(PRINT_CIRCLE_CENTRES == True):
                            plt.plot(self.wpnts_final_list[r][w][0],self.wpnts_final_list[r][w][1],'.',markersize=S_MARKERIZE,color=PATH_COLOR)
                        e1 = pat.Arc([self.wpnts_final_list[r][w][0],self.wpnts_final_list[r][w][1]],self.wpnts_final_list[r][w][2],self.wpnts_final_list[r][w][2],angle=self.wpnts_final_list[r][w][3],theta1=0.0,theta2=90.0,linewidth=LINEWIDTH,color=PATH_COLOR) # circle
                        ax.add_patch(e1)
                        circ_flag = 1

        # Plot robot initial positions
        plt.plot(self.rip_cont[r][1],self.rip_cont[r][0],'.k',markersize=int(MARKERSIZE))
        # plt.plot(self.rip_cont[r][1],self.rip_cont[r][0],'.w',markersize=int(MARKERSIZE/3))

        # Plot target position
        plt.plot(self.t_x,self.t_y,'xk',markersize=int(MARKERSIZE))

    def prim_algorithm(self,graph,vertices):
        selected = np.zeros([vertices],dtype=bool)
        parents = np.zeros([vertices],dtype=int)
        weights = np.zeros([vertices])
        total_weight = 0
        selected[0] = True
        parents[0] = -1
        edges=0
        while(edges<vertices-1):
            minimum = np.inf
            for i in range(vertices):
                if(selected[i]):
                    for j in range(vertices):
                        if(not(selected[j]))and(graph[i][j]):
                            if(graph[i][j]<minimum):
                                minimum=graph[i][j] # update min
                                row = i 
                                col = j
            selected[col] = True
            parents[col] = row
            weights[col] = minimum
            total_weight+=minimum
            edges += 1
        return(parents)

# Prim Related
class mst_node:
    def __init__(self,i,x,y):
        self.node_number = i
        self.coord_x = x
        self.coord_y = y
    def set_edges(self,parents):
        self.edges = np.ones(4,dtype=int)*(-1)
        self.dir = np.ones(4,dtype=int)*(-1)
        edge_no = 0
        for node_ind in range(len(parents)):
            if(node_ind==self.node_number):
                self.edges[edge_no] = parents[node_ind]
                edge_no+=1
            elif(parents[node_ind]==self.node_number):
                self.edges[edge_no] = node_ind
                edge_no+=1
        self.no_edges = edge_no
        if(self.node_number==0):
            self.no_edges = self.no_edges-1 
    def reorganise_dir(self):
        dir_temp = np.ones(4,dtype=int)*-1
        edges_temp = np.ones(4,dtype=int)*-1
        for i in range(4):
            if(self.dir[i]==0):
                dir_temp[0] = self.dir[i]
                edges_temp[0] = self.edges[i]
            if(self.dir[i]==1):
                dir_temp[1] = self.dir[i]
                edges_temp[1] = self.edges[i]
            if(self.dir[i]==2):
                dir_temp[2] = self.dir[i]
                edges_temp[2] = self.edges[i]
            if(self.dir[i]==3):  
                dir_temp[3] = self.dir[i]
                edges_temp[3] = self.edges[i]
        for i in range(4):
            self.dir[i] = dir_temp[i]
            self.edges[i] = edges_temp[i]

class mst_arrow:
    def __init__(self,start_node,end_node,direction):
        self.start_node = start_node
        self.end_node = end_node
        self.direction = direction

# Environment grid creation
class generate_rand_grid:
    def __init__(self, rows, cols, robots, obs):
        self.rows = rows
        self.cols = cols
        self.GRID = np.zeros([rows, cols], dtype=int)
        self.n_r = robots
        self.obs = obs
        self.possible_indexes = np.argwhere(self.GRID == 0)
        np.random.shuffle(self.possible_indexes)
        self.es_flag = False

    def randomise_robots(self):
        if self.n_r < self.rows*self.cols:
            self.rip = self.possible_indexes[0:self.n_r]
            val1 = self.rip[:, 0]
            val2 = self.rip[:, 1]
            self.GRID[val1, val2] = 2
        else:
            print("MADNESS! Why do you have so many robots?")

    def randomise_obs(self):
        if self.obs < (self.rows*self.cols-self.n_r):
            indices = self.possible_indexes[self.n_r:self.n_r+self.obs]
            val1 = indices[:, 0]
            val2 = indices[:, 1]
            self.GRID[val1, val2] = 1
        else:
            print("MADNESS! Why so many obstacles?")

    def flag_enclosed_space(self):
        # Note that DARP re-does this, so co-ordinate them because enclosed_space_check is an expensive process
        ES = enclosed_space_check(
            self.n_r, self.rows, self.cols, self.GRID, self.rip)
        if ES.max_label > 1:
            self.es_flag = True

class generate_grid:
    def __init__(self,hor,vert):
        # Divide Environment Into Large Nodes
        self.rows = math.ceil(vert/(DISC_V*2))
        self.cols = math.ceil(hor/(DISC_H*2))
        self.GRID = np.zeros([self.rows, self.cols], dtype=int)
        self.possible_indexes = np.argwhere(self.GRID == 0)
        np.random.shuffle(self.possible_indexes)
    def set_robots(self,n_r,coords):
        # Function not recently tested - might not work
        self.n_r =  n_r # number of robots
        self.rip_cont = coords 
        self.rip_sml = np.zeros([len(coords),2],dtype=int)
        self.rip = np.zeros([len(coords),2],dtype=int)
        for r in range(self.n_r):
            rip = self.rip_cont[r]
            # small cell position and large cell position
            self.rip_sml[r][0] = math.floor(self.rip_cont[r][0]/DISC_V) # row
            self.rip_sml[r][1] = math.floor(self.rip_cont[r][1]/DISC_H) # col
            self.rip[r][0] = math.floor(self.rip_cont[r][0]/(DISC_V*2)) # row
            self.rip[r][1] = math.floor(self.rip_cont[r][1]/(DISC_H*2)) # col
            self.GRID[self.rip[r][0]][self.rip[r][1]] = 2
    def set_obs(self,obs_coords):
        for obs in obs_coords:
            self.GRID[obs[0]][obs[1]] = 1
    def randomise_robots(self,n_r):
        self.n_r = n_r
        self.rip_sml = np.zeros([n_r,2],dtype=int)
        self.rip_cont = np.zeros([n_r,2],dtype=float)
        # self.rip = np.zeros([n_r,2],dtype=int)
        if self.n_r < self.rows*self.cols:
            self.rip = self.possible_indexes[0:self.n_r]
            self.possible_indexes = np.delete(self.possible_indexes,np.arange(0,self.n_r,1),0)
            val1 = self.rip[:, 0]
            val2 = self.rip[:, 1]
            self.GRID[val1, val2] = 2
        else:
            print("MADNESS! Why do you have so many robots?")

        for r in range(self.n_r):
            self.rip_sml[r][0] = self.rip[r][0]*2
            self.rip_sml[r][1] = self.rip[r][1]*2
            self.rip_cont[r][0] = (self.rip_sml[r][0]+0.5)*DISC_V # vertical
            self.rip_cont[r][1] = (self.rip_sml[r][1]+0.5)*DISC_H # horizontal
    def randomise_obs(self,obs_perc):
        self.obs = math.floor(self.rows*self.cols*obs_perc/100)
        if self.obs < (self.rows*self.cols*0.75):
            indices = self.possible_indexes[0:self.obs]
            self.possible_indexes = np.delete(self.possible_indexes,np.arange(0,self.obs,1),0)
            val1 = indices[:, 0]
            val2 = indices[:, 1]
            self.GRID[val1, val2] = 1
        else:
            print("MADNESS! Why so many obstacles? More than 75%% seems a bit crazy.")
    def set_target(self,targ_coord):
        self.tp_cont = np.array(targ_coord,dtype=float)

if __name__ == "__main__":
    # Ensures it prints entire arrays when logging instead of going [1 1 1 ... 2 2 2]
    np.set_printoptions(threshold=np.inf)

## RUN AN INDIVIDUAL CASE -> CONTINUOUS SPACE##
    # Establish Environment Size - Chooses max horizontal and vertical dimensions and create rectangle
    horizontal = DISC_H*50 # m
    vertical = DISC_V*50 # m

    # Generate environment grid
    GG = generate_grid(horizontal,vertical)
    
    n_r = 10
    obs_perc = 0
    GG.randomise_robots(n_r) 
    GG.randomise_obs(obs_perc)
    GG.set_target([2000,2000]) # (vert,hor) from top left 

    # Other parameters
    Imp = False
    maxIter = 10000
    distance_measure = 0 # 0,1,2 - Euclidean, Manhattan, GeodisicManhattan
    
    rows = GG.rows
    cols = GG.cols
    dcells = math.ceil(rows*cols/10)
    
    print_graphs = True

    # RUNNING SIMULATION #
    file_log = "MAIN_LOGGING.txt"
    target_log = "TARGET_LOG.txt"
    EnvironmentGrid = GG.GRID

    #  Call this to do directory management and recompile Java files - better to keep separate for when running multple sims
    algorithm_start(recompile=False)
    
    # Call this to run DARP and MST
    RA = Run_Algorithm(EnvironmentGrid, GG.rip, dcells, Imp, print_graphs,dist_meas=distance_measure,log_active=False,log_filename=file_log,target_filename=target_log,target_active=True)
    RA.set_continuous(GG.rip_sml,GG.rip_cont, GG.tp_cont)
    RA.main()

    if print_graphs == True:
        plt.show()
