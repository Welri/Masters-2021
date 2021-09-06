import subprocess
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as pat
import matplotlib
import pathlib
import os
import random
import time
import math

# Constants
TREE_COLOR = 'k'
PATH_COLOR = 'w'
PRINT_DARP = True
PRINT_TREE = False
PRINT_PATH = True
PRINT_CIRCLE_CENTRES = False
LINEWIDTH = 0.5
S_MARKERIZE = LINEWIDTH*4
MARKERSIZE=LINEWIDTH*12
TICK_SPACING = 1
FIGURE_TITLE = "DARP Continuous Results"

# Mavic Air 2 Camera (Imagine )
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
    # GSD_w = Height * 100 * (sensor_height/10) / ((focal_length/10) * px_w) # cm/px

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

# Calculating actual values
FOV_H = Height * sensor_width / focal_length # result in m
FOV_V = AR*FOV_H # m
# Conservatively choose
# Note: for asymmetric cells FOV_V < FOV_H is a requirement
DISC_H = math.sqrt(2)/(4-math.sqrt(2)) * FOV_H
DISC_V = DISC_H
r_max = DISC_H/2
v_max = math.sqrt( r_max * g_acc * math.tan(phi_max*math.pi/180) ) # m/s
GSD_h = Height * 100 * (sensor_width/10) / ((focal_length/10) * px_h) # cm/px
GSD_w = Height * 100 * (sensor_height/10) / ((focal_length/10) * px_w) # cm/px
CT_Overlap = (DISC_V*(FOV_H -DISC_H)) / (DISC_V*DISC_H) # Crosstack overlap

ARC_L = DISC_V/2 - r_min + r_min*np.pi/2 + DISC_H/2 - r_min
FLIGHT_TIME = 30 * 60 # seconds

print("GSD of: ",GSD_h,"Overlap of: ", CT_Overlap)

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
    def __init__(self, EnvironmentGrid, rip, dcells, Imp, log_filename, show_grid=False,maxIter=10000,cc_vals=np.array([0.1,0.01,0.001,0.0001,0.00001]),rl_vals=np.array([0.01,0.001,0.0001,0.00001])):
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
        self.n_r = len(self.rip)
        self.ArrayOfElements = np.zeros(self.n_r, dtype=int)  
        self.abort = False
        self.es_flag = False
        self.runs = 0
        self.show_grid = show_grid
        self.DARP_success = False
        self.log_filename = log_filename
        self.total_iterations = 0

    def set_continuous(self,rip_sml,rip_cont):
        self.horizontal = 2*DISC_H*self.cols
        self.vertical = 2*DISC_V*self.rows
        self.rip_cont = rip_cont
        self.rip_sml = rip_sml
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
                    print("rl: ", self.rl, " cl: ", self.cc, " discrepancy allowed: ", self.dcells/self.fairDiv, "discrepancy achieved: ",self.maxDiscr)
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

    def primMST(self):
        try:
            # Run MST algorithm
            pMST = Prim_MST_maker(self.A,self.n_r,self.rows,self.cols,self.rip,self.Ilabel_final,self.rip_cont,self.rip_sml)

            # Print MST on DARP plot
            for r in range(self.n_r):
                pMST.waypoint_final_generation(pMST.free_nodes_list[r],pMST.parents_list[r],pMST.wpnts_cont_list[r],pMST.wpnts_class_list[r],self.ax,self.show_grid,r)
            print("Time allowed: ", FLIGHT_TIME )
            # print("Times achieved: ", pMST.time_totals)
            for r in range(self.n_r):
                time_ach = pMST.time_totals[r]
                if time_ach > FLIGHT_TIME:
                    print("Robot: ",r," Time Goal: ",FLIGHT_TIME," Time Achieved: ",time_ach," Exceeds Limit By ",time_ach - FLIGHT_TIME," seconds")
                else:
                    print("Robot: ",r," Time Goal: ",FLIGHT_TIME," Time Achieved: ",time_ach," Within Limit By ",FLIGHT_TIME - time_ach," seconds")

        except:
            print("Prim algorithm failed to implement...")
    
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
        if(GSD_h > 1.3):
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

        ax.set_xticks(np.arange(0, self.cols*2, step=TICK_SPACING),minor=False)
        ax.set_yticks(np.arange(0, self.rows*2, step=TICK_SPACING),minor=False)
        ax.set_xticks(np.arange(-0.5, self.cols*2+0.5, step=2),minor=True)
        ax.set_yticks(np.arange(-0.5, self.rows*2+0.5, step=2),minor=True)
        plt.xticks(rotation=90)

        plt.grid(which='minor',axis='both', color='k')

        # Print Assgnments
        for j in range(self.rows):
            for i in range(self.cols):
                x1 = (i-0.5)*2 + 0.5
                x2 = (i+0.5)*2 + 0.5
                y1 = (self.rows - (j-0.5) - 1)*2 + 0.5
                y2 = (self.rows - (j+0.5) - 1)*2 + 0.5
                if self.A[j][i] == self.n_r:
                    plt.fill([x1, x1, x2, x2], [y1, y2, y2, y1], "k")
                else:
                    plt.fill([x1, x1, x2, x2], [y1, y2, y2, y1],
                             colour_assignments[self.A[j][i]])

        plt.title("DARP Results")
    
    def cont_DARP_graph(self):
        plt.rc('font', size=12)
        plt.rc('axes', titlesize=15) 

        # Prints the DARP divisions
        fig,self.ax = plt.subplots(figsize=(8, 8))

        # Initialize cell colours
        # colours = ["C0", "C1", "C2", "C3", "C4", "C5", "C6", "C7", "C8", "C9","C10"]
        colours = ['xkcd:nice blue','xkcd:dusty orange','xkcd:kelly green','xkcd:red','xkcd:brownish','xkcd:carnation pink','xkcd:medium grey','xkcd:gold','xkcd:turquoise blue','xkcd:purple','xkcd:lightblue','xkcd:lavender','xkcd:salmon','xkcd:magenta','xkcd:olive','xkcd:silver']
        c = 0
        colour_assignments = {}
        for i in range(self.n_r):
            colour_assignments[i] = colours[c]
            c = c + 1
            if c == len(colours)-1:
                c = 0
            colour_assignments[self.n_r] = "k"

        # Ticks and Grid
        self.ax.set_xticks(np.arange(0, (self.cols*2+1)*DISC_H, step=2*DISC_H),minor=False) # Large cell grid-lines
        self.ax.set_yticks(np.arange(0, (self.rows*2+1)*DISC_V, step=2*DISC_V),minor=False)
        self.ax.set_xticks(np.arange(DISC_H, (self.cols*2-0.5)*DISC_H, step=2*DISC_H),minor=True) # Small cell grid_lines
        self.ax.set_yticks(np.arange(DISC_V, (self.rows*2-0.5)*DISC_V, step=2*DISC_V),minor=True)
        plt.xticks(rotation=90)
        plt.grid(which='major',axis='both', color='k',linewidth=1)
        plt.grid(which='minor',axis='both',color='k',linewidth=0.3)

        # Note: Robot initial position are printing in the prim algorithm

        # Print Assgnments
        for j in range(self.rows):
            for i in range(self.cols):
                x1 = ( (i-0.5)*2 + 0.5 + 0.5 )*DISC_H
                x2 = ( (i+0.5)*2 + 0.5 + 0.5 )*DISC_H
                y1 = ( (self.rows - (j-0.5) - 1)*2 + 0.5 + 0.5 )*DISC_V
                y2 = ( (self.rows - (j+0.5) - 1)*2 + 0.5 + 0.5 )*DISC_V
                if self.A[j][i] == self.n_r:
                    plt.fill([x1, x1, x2, x2], [y1, y2, y2, y1], "k")
                else:
                    plt.fill([x1, x1, x2, x2], [y1, y2, y2, y1],
                             colour_assignments[self.A[j][i]])

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
    def __init__(self,A,n_r,rows,cols,rip,Ilabel,rip_cont,rip_sml):
        self.A = A
        self.n_r = n_r
        self.rows = rows
        self.cols = cols
        self.rip = rip
        self.grids = Ilabel
        self.rip_cont = rip_cont
        self.rip_sml = rip_sml

        self.wpnts_final_list = list()
        self.dist_final_list = list()
        self.time_final_list = list()
        self.time_totals = np.zeros([self.n_r])
        self.dist_totals = np.zeros([self.n_r])

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

            # Final waypoints for last arrow
            self.wpnts_cont_list.append(self.waypoints_cont)
            self.wpnts_class_list.append(self.waypoint_class)
        
        # Shifting robot positions by half cell - p is found during waypoint generation
        for r in range(self.n_r):
            self.rip_cont[r] = self.wpnts_cont_list[r][self.p[r]]

        # Shifting waypoints to start at robot position and making it closed loop
        self.update_wpnts()
        
        # if print_graph == True:
        #     for r in range(self.n_r):
        #         self.draw_cont_graph(self.free_nodes_list[r],self.parents_list[r],self.wpnts_cont_list[r])
   
    def update_wpnts(self):
        for r in range(self.n_r):
            wpnts = self.wpnts_cont_list[r]
            wpnts_class = self.wpnts_class_list[r]
            wpnts_updated = np.zeros([len(wpnts)+1,2],dtype=float)
            wpnts_class_updated = np.zeros([len(wpnts_class)+1],dtype=float)
            p = self.p[r]
            ind = 0
            ind_p = p
            while(ind_p!=len(wpnts)):
                wpnts_updated[ind] = wpnts[ind_p]
                wpnts_class_updated[ind] = wpnts_class[ind_p]
                ind_p+=1
                ind+=1
            ind_p=0
            while(ind_p!=p):
                wpnts_updated[ind] = wpnts[ind_p]
                wpnts_class_updated[ind] = wpnts_class[ind_p]
                ind_p+=1
                ind+=1
            wpnts_updated[ind] = wpnts[p]
            wpnts_class_updated[ind] = wpnts_class[p]
            self.wpnts_cont_list[r] = wpnts_updated
            self.wpnts_class_list[r] = wpnts_class_updated

    # JAVA MST COMMENTED OUT - indented
        # def write_input(self,graph,dim):
        #     # write graphs to file
        #     file_in = open("MST_Input.txt","w")
        #     file_in.write(str(dim))
        #     file_in.write("\n")
        #     for i in range(dim):
        #         for j in range(dim):
        #             file_in.write(str(graph[i][j]))
        #             file_in.write("\n")
        #     file_in.close()
        
        # def run_subprocess(self):
        #     # Run the OS appropriate script to run the Java program for Prim's MST Algorithm for individual area searches
        #     if (os.name == 'nt'):
        #         # print("The current operating system is WINDOWS")
        #         subprocess.call([r'pMST_Run_Java.bat'])
        #     elif (os.name == 'posix'):
        #         # print("The current operating system is UBUNTU")
        #         subprocess.call("./pMST_Run_Java.sh")
        #     else:
        #         print("WARNING: Unrecognised operating system")
        
        # def read_output(self,dim):
        #     file_out = open("MST_Output.txt","r")
        #     parents = np.zeros(dim,dtype=int)
        #     for d in range(dim):
        #         parents[d] = int(file_out.readline())
        #     file_out.close()
        #     return(parents)

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
        # Add one waypoint
        direction = arrow.direction
        end_node = arrow.end_node
        X = end_node.coord_x
        Y = end_node.coord_y
        if(direction==0):
            # This means it went from E-N
            x = 2*X
            y_c = 2*Y + 1
            y = y_c + 0.5
            
            x = (x + 0.5)*DISC_H # added waypoint
            x_c = x
            y = (self.rows*2 - y - 0.5)*DISC_V # added waypoint 
            y_c = (self.rows*2 - y_c - 0.5)*DISC_V
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
        if(math.isclose(self.rip_cont[self.current_r][0],y_c))and(math.isclose(self.rip_cont[self.current_r][1],x_c)):
            self.p[self.current_r] = self.wpnt_ind
        self.waypoints_cont[self.wpnt_ind][0] = y
        self.waypoints_cont[self.wpnt_ind][1] = x
        self.waypoint_class[self.wpnt_ind] = 1
        self.wpnt_ind+=1
    
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
            # # parameters for arc
            # if(self.wpnt_ind==0):
            #     # if it is the first arrow - start position must be found manually
            #     start_x = 2*X + 1
            #     start_y = 2*Y + 2
            #     start_x = (start_x + 0.5)*FOV_H
            #     start_y = (self.rows*2 - start_y - 0.5)*FOV_V
            #     start = np.array([start_y,start_x])
            # else:
            #     start = self.waypoints_cont[self.wpnt_ind-1] # previous wpnt is start position
            # end = np.array([y3,x3])
            # a = -FOV_V # to make it the lower half the ellipse (a>b convention is not applied here)
            # b = FOV_H/2
            # y_offset = start[0]
            # x_offset = end[1] + ( (start[1]-end[1])/2 ) # start_x > end_x
            # # arc x values
            # x_arc = np.linspace(start[1],end[1],ARC_PNTS*2)
            # # arc y values
            # y_arc = y_offset + a * np.sqrt( np.abs( 1-((x_arc-x_offset)**2/b**2) ) )
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
            # # arc making
            # if(self.wpnt_ind==0):
            #     # if it is the first arrow - start position must be found manually
            #     start_x = 2*X - 1 
            #     start_y = 2*Y + 1
            #     start_x = (start_x + 0.5)*FOV_H
            #     start_y = (self.rows*2 - start_y - 0.5)*FOV_V
            #     start = np.array([start_y,start_x])
            # else:
            #     start = self.waypoints_cont[self.wpnt_ind-1] # previous wpnt is start position
            # end = np.array([y3,x3])
            # a = -FOV_H
            # b = FOV_V/2
            # x_offset = start[1]
            # y_offset = start[0] + (end[0]-start[0])/2
            # # arc y values
            # y_arc = np.linspace(start[0],end[0],ARC_PNTS*2)
            # # arc y values
            # x_arc = x_offset + a * np.sqrt( np.abs( 1-((y_arc-y_offset)**2/b**2) ) )
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
            # # arc making
            # if(self.wpnt_ind==0):
            #     # if it is the first arrow - start position must be found manually
            #     start_x = 2*X + 0
            #     start_y = 2*Y - 1
            #     start_x = (start_x + 0.5)*FOV_H
            #     start_y = (self.rows*2 - start_y - 0.5)*FOV_V
            #     start = np.array([start_y,start_x])
            # else:
            #     start = self.waypoints_cont[self.wpnt_ind-1] # previous wpnt is start position
            # end = np.array([y3,x3])
            # a = FOV_V
            # b = FOV_H/2
            # y_offset = start[0]
            # x_offset = start[1] + ( (end[1]-start[1])/2 ) # end_x > start_x
            # # arc x values
            # x_arc = np.linspace(start[1],end[1],ARC_PNTS*2)
            # # arc y values
            # y_arc = y_offset + a * np.sqrt( np.abs( 1-((x_arc-x_offset)**2/b**2) ) )
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
            # # arc making
            # if(self.wpnt_ind==0):
            #     # if it is the first arrow - start position must be found manually
            #     start_x = 2*X + 2
            #     start_y = 2*Y
            #     start_x = (start_x + 0.5)*FOV_H
            #     start_y = (self.rows*2 - start_y - 0.5)*FOV_V
            #     start = np.array([start_y,start_x])
            # else:
            #     start = self.waypoints_cont[self.wpnt_ind-1] # previous wpnt is start position
            # end = np.array([y3,x3])
            # a = FOV_H
            # b = FOV_V/2
            # x_offset = start[1]
            # y_offset = end[0] + (start[0]-end[0])/2
            #  # arc y values
            # y_arc = np.linspace(start[0],end[0],ARC_PNTS*2)
            # # arc y values
            # x_arc = x_offset + a * np.sqrt( np.abs( 1-((y_arc-y_offset)**2/b**2) ) )
        # # Add three points normally
        # for pnt in range(1,4):
        #     self.waypoints_cont[self.wpnt_ind] = np.array([y_arc[pnt],x_arc[pnt]])
        #     self.wpnt_ind+=1
        # # Insert other points - array only has space for four points
        # for pnt in range(4,ARC_PNTS*2):
        #     self.waypoints_cont = np.insert(self.waypoints_cont,self.wpnt_ind,np.array([y_arc[pnt],x_arc[pnt]]),axis=0)
        #     self.wpnt_ind+=1
        # Add final point normally
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
    
    def draw_graph(self,nodes,parents,wpnts):
        # NO LONGER IN USE
        # Draws graph on old DARP graph
        # Plot spanning tree
        for node in nodes:
            x = (node[1])*2 + 0.5
            y = (self.rows - node[0] - 1)*2 + 0.5
            plt.plot(x,y,".w")
        for i in range(1,len(parents)):
             x0 = (nodes[i][1])*2 + 0.5
             x1 = (nodes[parents[i]][1])*2 + 0.5
             y0 = (self.rows - nodes[i][0] - 1)*2 + 0.5
             y1 = (self.rows - nodes[parents[i]][0] - 1)*2 + 0.5
             plt.plot(np.array([x0,x1]),np.array([y0,y1]),"-w")

        # Plot waypoints
        px = np.zeros(len(wpnts),dtype=int)
        py = np.zeros(len(wpnts),dtype=int)
        for pi in range(len(wpnts)):
            py[pi] = self.rows*2 - wpnts[pi][0] - 1
            px[pi] = wpnts[pi][1]
        plt.plot(px,py,'-k') # linewidth=2
        plt.plot(px,py,'.k')
        for r in range(self.n_r):
            plt.plot(self.rip_sml[r][1],self.rows*2 - self.rip_sml[r][0] - 1,'.w',markersize=int(MARKERSIZE/3))
    
    def waypoint_final_generation(self,nodes,parents,wpnts,wpnts_class,ax,print_graph,r):
        # Waypoint, distance and time arrays created to describe the path
        # Includes graph drawing
        wpnts_final = list()
        dist_final = list()
        dist_final.append(0)
        dist_tot = 0
        time_tot = 0
        
        for w in range(len(wpnts)-1):
            x1 = wpnts[w][1]
            x2 = wpnts[w+1][1]
            y1 = wpnts[w][0]
            y2 = wpnts[w+1][0]
            if(x2>x1):
                if(y2>y1):
                    # F U
                    if(wpnts_class[w+1]==1):
                        # Bottom - Left
                        wpnts_final.append([x1,y1])
                        wpnts_final.append([x2-r_min,y1])
                        wpnts_final.append([x2-r_min,y1+r_min,2*r_min,270.0])
                        if(r_min != r_max):
                            wpnts_final.append([x2,y1+r_min])
                            wpnts_final.append([x2,y2])
                        l1 = DISC_H/2 - r_min # long leg
                        l2 = r_min*np.pi/2 # arc
                        l3 = DISC_V/2 - r_min # short leg
                        dist_final.append(l1)
                        dist_final.append(l2)
                        dist_final.append(l3)
                        dist_tot = dist_tot + l1 + l2 + l3
                    else:
                        # Top - Backtrack / Right
                        if(r_min != r_max):
                            wpnts_final.append([x1,y1])
                            wpnts_final.append([x1,y2-r_min])
                        wpnts_final.append([x1+r_min,y2-r_min,2*r_min,90.0])
                        wpnts_final.append([x1+r_min,y2])
                        wpnts_final.append([x2,y2])
                        l1 = DISC_V/2 - r_min # short leg 
                        l2 = r_min*np.pi/2 # arc
                        l3 = DISC_H/2 - r_min # long leg
                        dist_final.append(l1)
                        dist_final.append(l2)
                        dist_final.append(l3)
                        dist_tot = dist_tot + l1 + l2 + l3
                elif(y2<y1): 
                    # B D
                    if(wpnts_class[w+1]==1):
                        # Bottom - Left Turn
                        if(r_min != r_max):
                            wpnts_final.append([x1,y1])
                            wpnts_final.append([x1,y2+r_min])
                        wpnts_final.append([x1+r_min,y2+r_min,2*r_min,180.0])
                        wpnts_final.append([x1+r_min,y2])
                        wpnts_final.append([x2,y2])
                        l1 = DISC_V/2 - r_min # short leg 
                        l2 = r_min*np.pi/2 # arc
                        l3 = DISC_H/2 - r_min # long leg 
                        dist_final.append(l1)
                        dist_final.append(l2)
                        dist_final.append(l3)
                        dist_tot = dist_tot + l1 + l2 + l3              
                    else:
                        # Top - Right Turn / Backtrack
                        wpnts_final.append([x1,y1])
                        wpnts_final.append([x2-r_min,y1])
                        wpnts_final.append([x2-r_min,y1-r_min,2*r_min,0.0])
                        if(r_min != r_max):
                            wpnts_final.append([x2,y1-r_min])
                            wpnts_final.append([x2,y2])
                        l1 = DISC_H/2 - r_min # long leg
                        l2 = r_min*np.pi/2 # arc
                        l3 = DISC_V/2 - r_min # short leg
                        dist_final.append(l1)
                        dist_final.append(l2)
                        dist_final.append(l3)
                        dist_tot = dist_tot + l1 + l2 + l3
                else: # y2 == y1
                    # Line
                    wpnts_final.append([x1,y1])
                    wpnts_final.append([x2,y2])
                    dist_final.append(DISC_V)
                    dist_tot = dist_tot + DISC_V
            elif(x2<x1):
                if(y2>y1):
                    # B U
                    if(wpnts_class[w+1]==1):
                        # Top - Left Turn
                        if(r_min != r_max):
                            wpnts_final.append([x1,y1])
                            wpnts_final.append([x1,y2-r_min])
                        wpnts_final.append([x1-r_min,y2-r_min,2*r_min,0.0])
                        wpnts_final.append([x1-r_min,y2])
                        wpnts_final.append([x2,y2])                        
                        l1 = DISC_V/2 - r_min # short leg
                        l2 = r_min*np.pi/2 # arc
                        l3 = DISC_H/2 - r_min # long leg
                        dist_final.append(l1)
                        dist_final.append(l2)
                        dist_final.append(l3)
                        dist_tot = dist_tot + l1 + l2 + l3
                    else:
                        # Bottom - Right Turn / Backtrack
                        wpnts_final.append([x1,y1])
                        wpnts_final.append([x2+r_min,y1])
                        wpnts_final.append([x2+r_min,y1+r_min,2*r_min,180.0])
                        if(r_min != r_max):
                            wpnts_final.append([x2,y1+r_min])
                            wpnts_final.append([x2,y2])
                        l1 = DISC_H/2 - r_min # long leg
                        l2 = r_min*np.pi/2 # arc
                        l3 = DISC_V/2 - r_min # short leg
                        dist_final.append(l1)
                        dist_final.append(l2)
                        dist_final.append(l3)
                        dist_tot = dist_tot + l1 + l2 + l3
                elif(y2<y1):
                    # F D
                    if(wpnts_class[w+1]==1):
                        # Top - Left
                        wpnts_final.append([x1,y1])
                        wpnts_final.append([x2+r_min,y1])
                        wpnts_final.append([x2+r_min,y1-r_min,2*r_min,90.0])
                        if(r_min != r_max):
                            wpnts_final.append([x2,y1-r_min])
                            wpnts_final.append([x2,y2])
                        l1 = DISC_H/2 - r_min # long leg
                        l2 = r_min*np.pi/2 # arc
                        l3 = DISC_V/2 - r_min # short leg
                        dist_final.append(l1)
                        dist_final.append(l2)
                        dist_final.append(l3)
                        dist_tot = dist_tot + l1 + l2 + l3
                    else:
                        # Bottom - Right Turn / Backtrack      
                        if(r_min != r_max):
                            wpnts_final.append([x1,y1])
                            wpnts_final.append([x1,y2+r_min])
                        wpnts_final.append([x1-r_min,y2+r_min,2*r_min,270.0])
                        wpnts_final.append([x1-r_min,y2])
                        wpnts_final.append([x2,y2])
                        l1 = DISC_V/2 - r_min # short leg
                        l2 = r_min*np.pi/2 # arc
                        l3 = DISC_H/2 - r_min # long leg
                        dist_final.append(l1)
                        dist_final.append(l2)
                        dist_final.append(l3)
                        dist_tot = dist_tot + l1 + l2 + l3 
                else: # y2 == y1
                    # Line
                    wpnts_final.append([x1,y1])
                    wpnts_final.append([x2,y2])
                    dist_final.append(DISC_V)
                    dist_tot = dist_tot + DISC_V
            else: # x2 == x1
                # Line
                wpnts_final.append([x1,y1])
                wpnts_final.append([x2,y2])
                dist_final.append(DISC_H)
                dist_tot = dist_tot + DISC_H
        
        # Append to main lists
        self.wpnts_final_list.append(wpnts_final)
        self.dist_final_list.append(dist_final)
        self.dist_totals[r] = dist_tot
        time_final = np.zeros(len(dist_final))
        for d in range(len(dist_final)):
            time_final[d] = dist_final[d] / VEL
            time_tot = time_tot + time_final[d]
        self.time_final_list.append(time_final)
        self.time_totals[r] = time_tot
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
        
        # Plot waypoints
        if(PRINT_PATH==True):
            circ_flag = 0
            for w in range(1,len(wpnts_final)):
                if(len(wpnts_final[w])==2): # line
                    if(circ_flag == 0):    
                        plt.plot([wpnts_final[w-1][0],wpnts_final[w][0]],[wpnts_final[w-1][1],wpnts_final[w][1]],'-',linewidth=LINEWIDTH,color=PATH_COLOR)
                        plt.plot([wpnts_final[w-1][0],wpnts_final[w][0]],[wpnts_final[w-1][1],wpnts_final[w][1]],'.',markersize=S_MARKERIZE,color=PATH_COLOR)
                    else:
                        circ_flag = 0
                else: # circle
                    if(PRINT_CIRCLE_CENTRES == True):
                        plt.plot(wpnts_final[w][0],wpnts_final[w][1],'.',markersize=S_MARKERIZE,color=PATH_COLOR)
                    e1 = pat.Arc([wpnts_final[w][0],wpnts_final[w][1]],wpnts_final[w][2],wpnts_final[w][2],angle=wpnts_final[w][3],theta1=0.0,theta2=90.0,linewidth=LINEWIDTH,color=PATH_COLOR) # circle
                    ax.add_patch(e1)
                    circ_flag = 1

        # Plot robot initial positions
        plt.plot(self.rip_cont[r][1],self.rip_cont[r][0],'.k',markersize=int(MARKERSIZE))
        plt.plot(self.rip_cont[r][1],self.rip_cont[r][0],'.w',markersize=int(MARKERSIZE/3))

    def draw_cont_graph(self,nodes,parents,wpnts,wpnts_class,ax):
        # NO LONGER USING THIS
        # Draws graph on continuous space graph
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
                
        # Plot waypoints
        if(PRINT_PATH==True):
            for w in range(len(wpnts)-1):
                x1 = wpnts[w][1]
                x2 = wpnts[w+1][1]
                y1 = wpnts[w][0]
                y2 = wpnts[w+1][0]
                plt.plot(x1,y1,'.k',markersize=S_MARKERIZE,color=PATH_COLOR)
                if(x2>x1):
                    if(y2>y1):
                        # F U
                        if(wpnts_class[w+1]==1):
                            # Bottom - Left
                            plt.plot([x1,x2-r_min],[y1,y1],'-',linewidth=LINEWIDTH,color=PATH_COLOR)
                            # plt.plot(x2-r_min,y1,'.k',markersize=S_MARKERIZE)
                            if(PRINT_CIRCLE_CENTRES==True):
                                plt.plot(x2-r_min,y1+r_min,'.',markersize=S_MARKERIZE,color=PATH_COLOR)
                            e2 = pat.Arc([x2-r_min,y1+r_min],2*r_min,2*r_min,angle=270.0,theta1=0.0,theta2=90.0,linewidth=LINEWIDTH,color=PATH_COLOR)
                            if(r_min != r_max):
                                # plt.plot(x2,y1+r_min,'.k',markersize=S_MARKERIZE)
                                plt.plot([x2,x2],[y1+r_min,y2],'-',linewidth=LINEWIDTH,color=PATH_COLOR)
                            ax.add_patch(e2)
                        else:
                            # Top - Backtrack / Right
                            if(r_min != r_max):
                                plt.plot([x1,x1],[y1,y2-r_min],'-',linewidth=LINEWIDTH,color=PATH_COLOR) # line to start of circle
                            if(PRINT_CIRCLE_CENTRES==True):
                                plt.plot(x1+r_min,y2-r_min,'.',markersize=S_MARKERIZE,color=PATH_COLOR) # circle centre
                            e1 = pat.Arc([x1+r_min,y2-r_min],2*r_min,2*r_min,angle=90.0,theta1=0.0,theta2=90.0,linewidth=LINEWIDTH,color=PATH_COLOR) # circle
                            plt.plot([x1+r_min,x2],[y2,y2],'-',linewidth=LINEWIDTH,color=PATH_COLOR)# line from end of circle
                            ax.add_patch(e1)
                    elif(y2<y1): 
                        # B D
                        if(wpnts_class[w+1]==1):
                            # Bottom - Left Turn
                            if(r_min != r_max):
                                plt.plot([x1,x1],[y1,y2+r_min],'-',linewidth=LINEWIDTH,color=PATH_COLOR) # line to start of circle
                            if(PRINT_CIRCLE_CENTRES==True):
                                plt.plot(x1+r_min,y2+r_min,'.',markersize=S_MARKERIZE,color=PATH_COLOR) # circle centre
                            e1 = pat.Arc([x1+r_min,y2+r_min],2*r_min,2*r_min,angle=180.0,theta1=0.0,theta2=90.0,linewidth=LINEWIDTH,color=PATH_COLOR) # circle
                            plt.plot([x1+r_min,x2],[y2,y2],'-',linewidth=LINEWIDTH,color=PATH_COLOR)# line from end of circle
                            ax.add_patch(e1)
                        else:
                            # Top - Right Turn / Backtrack
                            plt.plot([x1,x2-r_min],[y1,y1],'-',linewidth=LINEWIDTH,color=PATH_COLOR)
                            if(PRINT_CIRCLE_CENTRES==True):
                                plt.plot(x2-r_min,y1-r_min,'.',markersize=S_MARKERIZE,color=PATH_COLOR)
                            e2 = pat.Arc([x2-r_min,y1-r_min],2*r_min,2*r_min,angle=0.0,theta1=0.0,theta2=90.0,linewidth=LINEWIDTH,color=PATH_COLOR)
                            if(r_min != r_max):
                                plt.plot([x2,x2],[y1-r_min,y2],'-',linewidth=LINEWIDTH,color=PATH_COLOR)
                            ax.add_patch(e2)
                    else: # y2 == y1
                        # Line
                        plt.plot([x1,x2],[y1,y2],'-',linewidth=LINEWIDTH,color=PATH_COLOR)
                elif(x2<x1):
                    if(y2>y1):
                        # B U
                        if(wpnts_class[w+1]==1):
                            # Top - Left Turn
                            if(r_min != r_max):
                                plt.plot([x1,x1],[y1,y2-r_min],'-',linewidth=LINEWIDTH,color=PATH_COLOR) # line to start of circle
                            if(PRINT_CIRCLE_CENTRES==True):
                                plt.plot(x1-r_min,y2-r_min,'.',markersize=S_MARKERIZE,color=PATH_COLOR) # circle centre
                            e1 = pat.Arc([x1-r_min,y2-r_min],2*r_min,2*r_min,angle=0.0,theta1=0.0,theta2=90.0,linewidth=LINEWIDTH,color=PATH_COLOR) # circle
                            plt.plot([x1-r_min,x2],[y2,y2],'-',linewidth=LINEWIDTH,color=PATH_COLOR)# line from end of circle
                            ax.add_patch(e1)
                        else:
                            # Bottom - Right Turn / Backtrack
                            plt.plot([x1,x2+r_min],[y1,y1],'-',linewidth=LINEWIDTH,color=PATH_COLOR)
                            if(PRINT_CIRCLE_CENTRES==True):
                                plt.plot(x2+r_min,y1+r_min,'.',markersize=S_MARKERIZE,color=PATH_COLOR)
                            e2 = pat.Arc([x2+r_min,y1+r_min],2*r_min,2*r_min,angle=180.0,theta1=0.0,theta2=90.0,linewidth=LINEWIDTH,color=PATH_COLOR)
                            if(r_min != r_max):
                                plt.plot([x2,x2],[y1+r_min,y2],'-',linewidth=LINEWIDTH,color=PATH_COLOR)
                            ax.add_patch(e2)
                    elif(y2<y1):
                        # F D
                        if(wpnts_class[w+1]==1):
                            # Top - Left
                            plt.plot([x1,x2+r_min],[y1,y1],'-',linewidth=LINEWIDTH,color=PATH_COLOR) # line to start of circle
                            if(PRINT_CIRCLE_CENTRES==True):
                                plt.plot(x2+r_min,y1-r_min,'.',markersize=S_MARKERIZE,color=PATH_COLOR) # circle centre
                            e2 = pat.Arc([x2+r_min,y1-r_min],2*r_min,2*r_min,angle=90.0,theta1=0.0,theta2=90.0,linewidth=LINEWIDTH,color=PATH_COLOR) # circle
                            if(r_min != r_max):    
                                plt.plot([x2,x2],[y1-r_min,y2],'-',linewidth=LINEWIDTH,color=PATH_COLOR)# line from end of circle
                            ax.add_patch(e2)
                        else:
                            # Bottom - Right Turn / Backtrack      
                            if(r_min != r_max):
                                plt.plot([x1,x1],[y1,y2+r_min],'-',linewidth=LINEWIDTH,color=PATH_COLOR) # line to start of circle
                            if(PRINT_CIRCLE_CENTRES==True):
                                plt.plot(x1-r_min,y2+r_min,'.',markersize=S_MARKERIZE,color=PATH_COLOR) # circle centre
                            e1 = pat.Arc([x1-r_min,y2+r_min],2*r_min,2*r_min,angle=270.0,theta1=0.0,theta2=90.0,linewidth=LINEWIDTH,color=PATH_COLOR) # circle
                            plt.plot([x1-r_min,x2],[y2,y2],'-',linewidth=LINEWIDTH,color=PATH_COLOR)# line from end of circle
                            ax.add_patch(e1)
                    else: # y2 == y1
                        # Line
                        plt.plot([x1,x2],[y1,y2],'-',linewidth=LINEWIDTH,color=PATH_COLOR)
                else: # x2 == x1
                    # Line
                    plt.plot([x1,x2],[y1,y2],'-',linewidth=LINEWIDTH,color=PATH_COLOR)

        # Plot robot initial positions
        for r in range(self.n_r):
            plt.plot(self.rip_cont[r][1],self.rip_cont[r][0],'.k',markersize=int(MARKERSIZE))
            plt.plot(self.rip_cont[r][1],self.rip_cont[r][0],'.w',markersize=int(MARKERSIZE/3))

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
            self.GRID[self.rip_lrg[r][0]][self.rip_lrg[r][1]] = 2
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

if __name__ == "__main__":
    # Ensures it prints entire arrays when logging instead of going [1 1 1 ... 2 2 2]
    np.set_printoptions(threshold=np.inf)

## RUN AN INDIVIDUAL CASE -> CONTINUOUS SPACE##
    # Establish Environment Size - Chooses max horizontal and vertical dimensions and create rectangle
    horizontal = 1500.0 # m
    vertical = 1500.0 # m

    # Establish Small Node size
    
    GG = generate_grid(horizontal,vertical)
    
    # Coordinates from top left (vert,hor)
    n_r = 3
    obs_perc = 10
    GG.randomise_robots(n_r)
    GG.randomise_obs(obs_perc)
    # robot_cont = np.array([[50,160],[180,40]])
    # GG.set_robots(n_r,robot_cont)
    # obs = np.array([[0,1],[0,2],[1,1],[1,2],[2,2],[2,3],[2,4],[2,5],[3,4],[3,5],[7,4],[7,5],[8,4],[8,5],[9,5]])
    # GG.set_obs(obs)

    # Other parameters
    Imp = False
    maxIter = 10000
    
    rows = GG.rows
    cols = GG.cols
    dcells = math.ceil(rows*cols/10)
    
    print_graphs = True

    # RUNNING SIMULATION #
    file_log = "Logging_005.txt"
    EnvironmentGrid = GG.GRID

    #  Call this to do directory management and recompile Java files - better to keep separate for when running multiple sims
    algorithm_start(recompile=True)

    # Call this to run DARP and MST
    RA = Run_Algorithm(EnvironmentGrid, GG.rip, dcells, Imp, file_log, print_graphs)
    RA.set_continuous(GG.rip_sml,GG.rip_cont)
    RA.main()

    if print_graphs == True:
        plt.show()
