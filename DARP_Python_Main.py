import subprocess
import numpy as np
import matplotlib.pyplot as plt
import pathlib
import os
import random
import time

class DARP:
    def __init__(self, EnvironmentGrid, dcells, Imp, log_filename, show_grid=False,maxIter=10000,cc_vals=np.array([0.1,0.01,0.001]),rl_vals=np.array([0.01,0.001,0.0001])):
        # DIRECTORY MANAGEMENT
        path = pathlib.Path(__file__).parent.absolute()
        # Changing current working directory to directory this file is in (avoid directory conflict with subprocess)
        os.chdir(path)
        # print("CURRENT WORKING DIRECTORY:", os.getcwd())

        self.Grid = EnvironmentGrid
        self.maxIter = maxIter
        self.dcells = dcells
        self.cc_vals = cc_vals#np.array([0.1, 0.01, 0.001])
        # self.cc = self.cc_vals[0]
        self.rl_vals = rl_vals#np.array([0.01, 0.001, 0.0001])
        # self.rl = self.rl_vals[0]
        self.Imp = Imp
        self.rows = len(self.Grid)
        self.cols = len(self.Grid[0])
        self.rip = np.argwhere(self.Grid == 2)
        self.n_r = len(self.rip)
        self.ArrayOfElements = np.zeros(self.n_r, dtype=int)
        self.fairDiv = self.rows*self.cols/self.n_r
        self.abort = False
        self.es_flag = False
        self.runs = 0
        self.show_grid = show_grid
        self.DARP_success = False
        self.log_filename = log_filename
        self.total_iterations = 0

    def main_DARP(self):
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
                    self.AOEperc = np.abs(
                        (self.ArrayOfElements+1)-self.fairDiv)/self.fairDiv
                    self.maxDiscr = np.max(self.AOEperc)
                    if self.show_grid == True:
                        self.print_DARP_graph()
                    self.runs += 1
                    self.total_iterations = self.total_iterations + self.iterations
                    if self.DARP_success == True:
                        break
                else:
                    continue
                break

        elif self.abort == True:
            # Cancels program if any errors occurred in error handling
            # Calculate the obstacles seen as they aren't returned by Java algorithm
            self.obs = len(np.argwhere(self.Grid == 1))
            print("Aborting Algorithm...")

        self.time_elapsed = time.time_ns() - timestart

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
            file_log.write(str(self.time_elapsed))
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
            file_log.write(str(self.time_elapsed))
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
        # Runs the OS appropriate script to run the java program
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
        # Prints the DARP divisions
        plt.figure(figsize=(5, 5))

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

        ripy, ripx = zip(*self.rip)
        ripy = self.rows - np.array(ripy)
        plt.plot(ripx, ripy, 'xk', markersize=20)

        # Print Assgnments
        for j in range(self.rows):
            for i in range(self.cols):
                x1 = i-0.5
                x2 = i+0.5
                y1 = self.rows - (j-0.5)
                y2 = self.rows - (j+0.5)
                if self.A[j][i] == self.n_r:
                    plt.fill([x1, x1, x2, x2], [y1, y2, y2, y1], "k")
                else:
                    plt.fill([x1, x1, x2, x2], [y1, y2, y2, y1],
                             colour_assignments[self.A[j][i]])

        plt.title("Figure generated from DARP run")
        # plt.show()

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

class generate_grid:
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

if __name__ == "__main__":
    # Ensures it prints entire arrays when logging instead of going [1 1 1 ... 2 2 2]
    np.set_printoptions(threshold=np.inf)

## RUN MULTIPLE SIMULATIONS ##

    # FIXED PARAMETERS #
    Imp = False
    maxIter = 10000
    dcells = 30
    cc = 0.1
    rl = 0.0001
    obs_perc = 20

    # Number of simulations per case
    number_of_sims = 10

    # VARIABLES #
    # Grid size range 
    # OLD WAY OF DOING IT- INDENTED
        # max_size = 98
        # min_size = 98
        # step_size = 10
        # sizes = np.array(np.arange(min_size/step_size, max_size /
        #                            step_size + 1), dtype=int)*step_size
    sizes = np.array([58])
    rows = sizes
    cols = sizes

    # Number of robots range
    # OLD WAY OF DOING IT - INDENTED
        # min_robots = 2
        # max_robots = 6
        # step_robots = 1
        # robots = np.array(np.arange(min_robots/step_robots,max_robots/step_robots+1),dtype=int)*step_robots
    robots = np.array([2, 8, 14, 20])
    # CALCULATING TOTAL SIMULATIONS AND PRINTING #
    sims = number_of_sims*len(rows)*len(cols)*len(robots)
    print("Number of Sims Total: ", sims)

    # RUNNING SIMULATIONS #
    sim_overall = 1
    file_log = "Logging_003.txt"
    FL = open(file_log, "a")
    FL.write("Running Zeng style test with  obstacles\n\n")
    FL.close()
    for r in rows:
        for c in cols:
            for robot in robots:
                for sim in range(number_of_sims):
                    print("SIMULATION: ", sim_overall)
                    sim_overall = sim_overall + 1
                    obstacles = int((r*c)*obs_perc/100)
                    grid_class = generate_grid(r, c, robot, obstacles)
                    grid_class.randomise_robots()
                    grid_class.randomise_obs()

                    EnvironmentGrid = grid_class.GRID
                    print_graph = False

                    dp = DARP(EnvironmentGrid, dcells, Imp, file_log, print_graph)
                    dp.main_DARP()
                    if print_graph == True:
                        plt.show()

## RUN AN INDIVIDUAL CASE ##

    # Imp = False

    # maxIter = 1000
    # dcells = 30
    # cc = 0.01
    # rl = 0.0001
    # r = 10
    # c = 10
    # robot = 3
    # obstacles = 0

    # number_of_sims = 1

    # sim_overall = 1

    # for sim in range(number_of_sims):
    #     print("SIMULATION: ", sim_overall)
    #     sim_overall = sim_overall + 1
    #     grid_class = generate_grid(r, c, robot, obstacles)
    #     grid_class.randomise_robots()
    #     grid_class.randomise_obs()
    #     # grid_class.flag_enclosed_space()
    #     # print(grid_class.GRID)
    #     # print(grid_class.es_flag)

    #     EnvironmentGrid = grid_class.GRID
    #     print_graph = False
    #     dp = DARP(EnvironmentGrid, maxIter, dcells,
    #               cc, rl, Imp, "Logging.txt", print_graph)
    #     dp.main_DARP()
    #     if print_graph == True:
    #         plt.show()
