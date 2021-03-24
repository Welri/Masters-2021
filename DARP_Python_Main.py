import subprocess
import numpy as np
import matplotlib.pyplot as plt
import pathlib
import os
import random


## DIRECTORY MANAGEMENT
path = pathlib.Path(__file__).parent.absolute() 
os.chdir(path)  # Changing current working directory to directory this file is in (avoid directory conflict with subprocess) 
print("CURRENT WORKING DIRECTORY:",os.getcwd()) 

class DARP:
    def __init__(self,EnvironmentGrid,maxIter,dcells,cc,rl,Imp):
        self.Grid = EnvironmentGrid
        self.maxIter = maxIter
        self.dcells = dcells
        self.cc = cc
        self.rl = rl
        self.Imp = Imp
        self.rows = len(self.Grid)
        self.cols = len(self.Grid[0])
        self.rip = np.argwhere(self.Grid==2)
        self.n_r = len(self.rip)  
        self.abort = False
    def main_DARP(self):
        self.enclosed_space_handler()
        self.general_error_handling()
        if self.abort == False:
            self.write_input()
            self.run_subprocess()
            self.read_output()
            self.print_DARP_graph()
        elif self.abort == True:
            ## Cancels program if any errors occurred in error handling
            print("Aborting Algorithm...")
    def enclosed_space_handler(self):
        ## Enclosed spaces (unreachable areas) are classified as obstacles
        ES = enclosed_space_check(self.n_r,self.rows,self.cols,self.Grid,self.rip)
        if ES.max_label > 1:
            print("WARNING: Automatic removal of enclosed space (it is considered an obstacle) ....")
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
                self.rip = np.argwhere(self.Grid==2)
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
    def write_input(self):
        ## Writes relevant inputs for java code to file
        # print(pathlib.Path("Input.txt").absolute())
        # file_in = open("DARP_Java/Input.txt", "w")
        file_in = open("Input.txt", "w")
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
        ## Runs the OS appropriate script to run the java program
        # subprocess.call([r'DARP_Java\Run_Java.bat'])
        # print(pathlib.Path('Run_Java.bat').absolute())
        if (os.name == 'nt'):
            print("The current operating system is WINDOWS")
            subprocess.call([r'Run_Java.bat'])
        elif (os.name == 'posix'):
            print("The current operating system is UBUNTU")
            subprocess.call("./Run_Java.sh")
        else:
            print("WARNING: Unrecognised operating system")
    def read_output(self):
        ## Read file containing outputs that were wrote to file by Java file
        self.A = np.zeros([self.rows, self.cols])
        # print(pathlib.Path("Output_A.txt").absolute())
        # file_out = open("DARP_Java/Output_A.txt", "r")
        file_out = open("Output_A.txt", "r")
        for i in range(self.rows):
            for j in range(self.cols):
                self.A[i][j] = int(file_out.readline())
        file_out.close()
    def print_DARP_graph(self):
        ## Prints the DARP divisions
        plt.figure(figsize=(5,5))
        # Initialize cell colours
        colours = ["C0", "C1", "C2", "C3", "C4", "C5","C6","C7","C8","C9"]
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
        # plt.show()

class enclosed_space_check:
    def __init__(self,n_r,n_rows,n_cols,EnvironmentGrid,rip):
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
    def transform2Dto1D(self,array2D):
        length = len(array2D)*len(array2D[0])
        array1D = np.reshape(array2D,length)
        return(array1D)
    def labeling(self,bin1D):
        rst = np.zeros([self.n_rows*self.n_cols], dtype=int)
        self.parent = np.zeros([self.MAX_LABELS], dtype=int)
        self.labels = np.zeros([self.MAX_LABELS])

        next_region = 1
        for i in range(self.n_rows): # height
            for j in range(self.n_cols): # width
                if (bin1D[i*self.n_cols+j] == 0):
                    continue
                k = 0
                connected = False
               
                # Check if connected to the left
                if ( (j > 0) and (bin1D[i*self.n_cols+j-1] == bin1D[i*self.n_cols+j]) ):
                    k = rst[i*self.n_cols+j-1]
                    connected = True
                # Check if connected to the top
                if ( (i > 0) and (bin1D[(i-1)*self.n_cols+j] == bin1D[i*self.n_cols+j]) and ( (connected == False) or ( bin1D[(i-1)*self.n_cols+j] < k ) ) ):
                    k = rst[(i-1)*self.n_cols+j]
                    connected = True
                if(connected == False):
                    k = next_region
                    next_region += 1

                rst[i*self.n_cols+j] = k
                if ( (j > 0) and (bin1D[i*self.n_cols+j-1] == bin1D[i*self.n_cols+j]) and  rst[i*self.n_cols+j-1] != k ):
                    self.uf_union(k,rst[i*self.n_cols+j-1])
                if ( (i > 0) and (bin1D[(i-1)*self.n_cols+j] == bin1D[i*self.n_cols+j]) and rst[(i-1)*self.n_cols+j] != k ):
                    self.uf_union(k,rst[(i-1)*self.n_cols+j])
        
        self.next_label = 1
        for ind in range(self.n_cols*self.n_rows):
            if ( (bin1D[ind] != 0) ):
                rst[ind] = self.uf_find( rst[ind] )
        self.next_label -= 1
        return(rst)
    def compact_labeling(self):
        bin_1D = self.transform2Dto1D(self.binary_grid)
        self.MAX_LABELS = self.n_rows * self.n_cols
        
        # Label the different regions
        label1D = self.labeling(bin_1D)
        self.max_label = self.next_label

        stat = np.zeros(self.max_label+1)
        for l in range(0,self.max_label+1):
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
        
        label2D = np.reshape(label1D,[self.n_rows,self.n_cols])
        return(label2D)
    def uf_union(self,a,b):
        a = int(a)
        b = int(b)
        while (self.parent[a] > 0):
            a = self.parent[a]
        while (self.parent[b] > 0):
            b = self.parent[b]
        if ( a != b ):
            if ( a > b ):
                self.parent[a] = b
            else:
                self.parent[b] = a
    def uf_find(self,r):
        while ( self.parent[r] > 0 ):
            r = self.parent[r]
        if ( self.labels[r] == 0 ):
            self.labels[r] = self.next_label
            self.next_label += 1
        return self.labels[r]

class generate_grid:
    def __init__(self,rows,cols,robots,obs):
        self.rows = rows
        self.cols = cols
        self.GRID = np.zeros([rows,cols])
        self.n_r = robots
        self.obs = obs
        self.possible_indexes = np.argwhere(self.GRID==0)
        np.random.shuffle(self.possible_indexes)
    def randomise_robots(self):
        if self.n_r < self.rows*self.cols:
            indices=self.possible_indexes[0:self.n_r]
            val1 = indices[:,0]
            val2 = indices[:,1]
            self.GRID[val1,val2] = 2
        else:
            print("MADNESS! Why do you have so many robots?")
    def randomise_obs(self):
        if self.obs < (self.rows*self.cols-self.n_r):
            indices = self.possible_indexes[self.n_r:self.n_r+self.obs]
            val1 = indices[:,0]
            val2 = indices[:,1]
            self.GRID[val1,val2] = 1
        else:
            print("MADNESS! Why so many obstacles?")

if __name__ == "__main__":
    maxIter = 1000
    dcells = 30
    cc = 0.01
    rl = 0.0001
    # EnvironmentGrid = np.array([[0,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    #                             [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,0,0,0,0,0,0,0,0],
    #                             [0,1,0,0,0,0,0,0,0,0,0,0,0,2,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    #                             [0,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    #                             [0,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    #                             [0,1,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    #                             [1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    #                             [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,0,0,0,0,0],
    #                             [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    #                             [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    #                             [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    #                             [1,1,0,0,0,0,0,0,0,0,0,2,0,0,0,0,0,0,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    #                             [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    #                             [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0],
    #                             [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0],
    #                             [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0],
    #                             [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0],
    #                             [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0],
    #                             [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0],
    #                             [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    #                             [1,1,0,0,0,0,0,0,2,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    #                             [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    #                             [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    #                             [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    #                             [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    #                             [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    #                             [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
    #                             [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,2,0,0,0],
    #                             [1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]])
    # EnvironmentGrid = np.array([[0,0,0,0,2],
    #                             [0,1,1,1,0],
    #                             [0,1,0,1,0],
    #                             [0,1,0,1,0],
    #                             [1,1,1,2,0],
    #                             [1,1,0,0,0]])
    # EnvironmentGrid = np.array([[0,0,0,0,0,2],
    #                             [0,0,1,1,0,0],
    #                             [0,1,0,1,1,0],
    #                             [0,1,1,1,1,0],
    #                             [0,0,0,1,0,0],
    #                             [0,0,0,2,0,0]])

    # EnvironmentGrid = np.array([[0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0 ,0,0,0,0,0, 0,0,0,0,0],
    #                             [0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0 ,0,0,0,0,0, 0,0,0,0,0],
    #                             [0,0, 0,0,0,0,0, 0,0,0,0,0, 1,1,1,1,0, 0,0,0,0,0 ,0,1,1,0,0, 0,0,0,0,0],
    #                             [0,0, 0,0,0,0,0, 0,0,0,0,0, 1,0,0,1,0, 0,0,0,0,0 ,1,1,1,1,0, 0,0,0,0,0],
    #                             [0,0, 0,0,0,0,0, 0,0,0,0,0, 1,1,0,1,0, 0,0,0,0,0 ,1,1,1,1,0, 0,0,0,0,0],

    #                             [0,0, 0,0,0,0,0, 2,0,0,0,0, 0,1,0,1,0, 0,0,0,0,0 ,0,0,1,1,1, 0,0,0,0,0],
    #                             [0,0, 0,0,0,0,0, 0,0,0,0,0, 0,1,0,1,0, 0,0,0,0,0 ,0,0,1,0,1, 0,0,0,0,0],
    #                             [0,0, 0,0,0,0,0, 0,0,0,0,0, 0,1,0,1,0, 0,0,0,0,0 ,0,1,1,0,1, 0,0,0,0,0],
    #                             [0,0, 0,0,0,0,0, 0,0,0,0,0, 0,1,1,1,0, 0,0,0,0,0 ,0,0,1,1,1, 0,0,0,0,0],
    #                             [0,0, 0,0,0,0,0, 0,0,0,0,0, 0,1,1,0,0, 0,0,0,0,0 ,0,0,0,1,1, 0,0,0,0,0],
                                
    #                             [0,0, 0,0,0,0,0, 0,0,0,0,0, 1,1,1,0,0, 0,0,0,0,0 ,0,0,0,0,0, 0,0,0,0,0],
    #                             [0,0, 0,0,0,0,0, 0,0,0,0,0, 1,1,1,0,0, 0,0,0,0,0 ,0,0,0,0,0, 0,0,0,0,0],
    #                             [0,0, 0,0,0,0,0, 0,0,0,0,0, 1,1,1,0,0, 0,0,0,0,0 ,0,0,0,0,0, 0,0,0,0,0],
    #                             [0,0, 0,0,0,0,0, 0,0,0,1,1, 1,1,1,1,1, 0,0,0,0,0 ,0,0,0,0,0, 0,0,0,0,0],
    #                             [0,0, 0,0,0,0,0, 1,1,1,1,1, 1,1,1,1,1, 0,0,0,0,0 ,0,0,0,0,0, 0,0,0,0,0],
                                
    #                             [0,0, 0,0,0,1,1, 1,0,0,1,1, 0,0,0,1,1, 0,0,0,0,0 ,0,1,0,0,0, 0,0,0,0,0],
    #                             [0,0, 0,0,0,1,0, 0,0,0,1,1, 0,0,0,1,1, 0,0,1,0,0 ,0,1,0,0,0, 0,0,0,0,0],
    #                             [0,0, 0,0,1,1,1, 0,0,0,1,1, 0,0,0,1,1, 1,1,1,1,0 ,1,1,0,0,0, 0,0,0,0,0],
    #                             [0,0, 0,0,0,1,1, 0,0,0,1,1, 1,1,1,1,1, 1,1,1,1,1 ,1,1,0,0,0, 0,0,0,0,0],
    #                             [0,0, 0,0,1,1,1, 1,1,1,1,1, 0,0,0,0,1, 0,0,0,1,1 ,1,0,0,0,0, 0,0,0,0,0],
                                
    #                             [1,1, 1,1,1,1,1, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0 ,0,0,0,0,0, 0,0,0,0,0],
    #                             [1,1, 1,1,1,1,1, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0 ,0,0,0,0,0, 0,0,0,0,0],
    #                             [1,1, 1,1,1,1,1, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0 ,0,0,0,0,0, 0,0,0,0,0],
    #                             [1,1, 1,1,1,1,1, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0 ,0,0,0,0,0, 0,0,0,0,0],
    #                             [1,1, 1,1,1,1,1, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,2,0 ,0,0,0,0,0, 0,0,0,0,0],
                                
    #                             [1,1, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0 ,0,0,0,0,0, 0,0,0,1,1],
    #                             [1,1, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0 ,0,0,0,0,0, 0,0,1,0,0],
    #                             [1,1, 0,0,2,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0 ,0,0,1,1,1, 1,1,0,0,0],
    #                             [1,1, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0 ,0,1,0,0,0, 0,0,0,0,0],
    #                             [1,1, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0 ,1,0,0,0,0, 0,0,0,0,0],
                                
    #                             [0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0 ,1,0,0,0,0, 0,0,0,0,0],
    #                             [0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0, 0,0,0,0,0 ,1,0,0,0,0, 0,0,0,0,0]])
    # Imp = False

    # dp = DARP(EnvironmentGrid,maxIter,dcells,cc,rl,Imp)
    # dp.main_DARP()
    
    # plt.show()

    grid_class = generate_grid(10,10,3,50)
    grid_class.randomise_robots()
    grid_class.randomise_obs()
    print(grid_class.GRID)

    # need to check for trapped robots
    # need to remember that filling up enclosed spaces can remove robots AND/OR increase obstacles 
    # so those cases make a simulation's parameters different
