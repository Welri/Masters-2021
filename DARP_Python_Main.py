import subprocess
import numpy as np
import matplotlib.pyplot as plt
import pathlib
import os
import random
import time

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

class Run_Algorithm:
    def __init__(self, EnvironmentGrid, dcells, Imp, log_filename, show_grid=False,maxIter=10000,cc_vals=np.array([0.1,0.01,0.001]),rl_vals=np.array([0.01,0.001,0.0001])):
        # # DIRECTORY MANAGEMENT
        # path = pathlib.Path(__file__).parent.absolute()
        # # Changing current working directory to directory this file is in (avoid directory conflict when running subprocesses)
        # os.chdir(path)
        # # print("CURRENT WORKING DIRECTORY:", os.getcwd())

        # # Compile all the Java programs
        # if recompile_java == True:   
        #     self.compilation_subprocess()

        self.Grid = EnvironmentGrid
        self.maxIter = maxIter
        self.dcells = dcells
        self.cc_vals = cc_vals
        self.rl_vals = rl_vals
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

    # PRIM MST SECTION
        self.primMST()

    def primMST(self):
        pMST = Prim_MST_maker(self.A,self.n_r,self.rows,self.cols,self.rip,self.Ilabel_final,self.show_grid)

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
        # Prints the DARP divisions
        plt.figure(figsize=(8, 8))

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
        ripx = (np.array(ripx))*2
        ripy = (self.rows - np.array(ripy) - 1)*2
        plt.plot(ripx, ripy, '.w', markersize=15)
        plt.grid(which='major',axis='both', color='k')
        plt.xticks(np.arange(0, self.cols*2, step=1))
        plt.yticks(np.arange(0, self.rows*2, step=1))

        # Print Assgnments
        for j in range(self.rows):
            for i in range(self.cols):
                x1 = (i-0.5)*2
                x2 = (i+0.5)*2
                y1 = (self.rows - (j-0.5) - 1)*2
                y2 = (self.rows - (j+0.5) - 1)*2
                if self.A[j][i] == self.n_r:
                    plt.fill([x1, x1, x2, x2], [y1, y2, y2, y1], "k")
                else:
                    plt.fill([x1, x1, x2, x2], [y1, y2, y2, y1],
                             colour_assignments[self.A[j][i]])

        plt.title("Figure generated from DARP run")

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

class Prim_MST_maker:
    def __init__(self,A,n_r,rows,cols,rip,Ilabel,print_graph):
        self.A = A
        self.n_r = n_r
        self.rows = rows
        self.cols = cols
        self.rip = rip
        self.grids = Ilabel
        self.small_cell_grids = np.zeros([self.n_r,self.rows*2,self.cols*2])

        # Grids: 0 is obstacle, 1 is free space
        # Graphs represent each individual node and which nodes it is connected to
        moves = np.array([[0,1],[1,0],[0,-1],[-1,0]])
        self.free_nodes_list = list()
        self.vertices_list = list()
        self.parents_list = list()
        self.nodes_list = list()
        self.wpnts_list = list()
        # self.arw_list = list()
        
        for r in range(self.n_r):   
            free_nodes = np.argwhere(self.grids[r]==1) # vertice coordinates
            vertices = len(free_nodes) # number of vertices

            graph = np.zeros([vertices,vertices],dtype = int)
            parents = np.zeros(vertices,dtype = int) 
            node_ind = 0
            for node_ind in range(vertices):
                node = free_nodes[node_ind]
                for move in moves:
                    row = node[0]+move[0]
                    col = node[1]+move[1]
                    if(row>=0)and(col>=0)and(row<self.rows)and(col<self.cols):
                        neighbour_node_ind = np.argwhere((free_nodes==(row,col)).all(axis=1))
                        if len(neighbour_node_ind)>0:
                            # Add edge to graph
                            graph[node_ind][neighbour_node_ind[0][0]]=1 # weights are just 1 for now

            self.write_input(graph,vertices)
            self.run_subprocess()
            parents = self.read_output(vertices)

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
            self.waypoints = np.empty([vertices*4,2],dtype=mst_node)
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

            # Update Arrow
            while(arw_ind!=no_arws):
                arrow = self.update_arrow(nodes,arrow.end_node,arrow.direction)
                arrows[arw_ind] = arrow
                arw_ind += 1

            # Final waypoints for last arrow


            self.wpnts_list.append(self.waypoints)
            # self.arw_list.append(arrows)
            
        if print_graph == True:
            for r in range(self.n_r):
                self.small_cell_grids[r] = self.small_cell_grid(self.grids[r],self.rows,self.cols)
                self.draw_graph(self.free_nodes_list[r],self.parents_list[r])
   
    def write_input(self,graph,dim):
        # write graphs to file
        file_in = open("MST_Input.txt","w")
        file_in.write(str(dim))
        file_in.write("\n")
        for i in range(dim):
            for j in range(dim):
                file_in.write(str(graph[i][j]))
                file_in.write("\n")
        file_in.close()
    
    def run_subprocess(self):
        # Run the OS appropriate script to run the Java program for Prim's MST Algorithm for individual area searches
        if (os.name == 'nt'):
            # print("The current operating system is WINDOWS")
            subprocess.call([r'pMST_Run_Java.bat'])
        elif (os.name == 'posix'):
            # print("The current operating system is UBUNTU")
            subprocess.call("./pMST_Run_Java.sh")
        else:
            print("WARNING: Unrecognised operating system")
    
    def read_output(self,dim):
        file_out = open("MST_Output.txt","r")
        parents = np.zeros(dim,dtype=int)
        for d in range(dim):
            parents[d] = int(file_out.readline())
        file_out.close()
        return(parents)

    def small_cell_grid(self,grid,rows,cols):
        SC_grid = np.zeros([rows*2,cols*2],dtype=int)
        for i in range(rows):
            for j in range(cols):
                i_SC = i*2 # i starting index for bigger grid
                j_SC = j*2 # j starting index for bigger grid
                val = grid[i][j]
                # Assign one grid value to every four cells
                SC_grid[i_SC][j_SC] = val
                SC_grid[i_SC+1][j_SC] = val
                SC_grid[i_SC][j_SC+1] = val
                SC_grid[i_SC+1][j_SC+1] = val
        return(SC_grid)

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
            y = 2*Y + 1
        elif(direction==1):
            # This means it went from S-E
            x = 2*X
            y = 2*Y
        elif(direction==2):
            # This means it went from W-S
            x = 2*X + 1
            y = 2*Y
        elif(direction==3):        
            # This means it went from N-W
            x = 2*X + 1
            y = 2*Y + 1
        self.waypoints[self.wpnt_ind][0] = y # row
        self.waypoints[self.wpnt_ind][1] = x # col
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
            y1 = 2*Y + 2
            x2 = 2*X
            y2 = 2*Y + 1
        elif(direction==1):
            # This means it went from E-E
            x1 = 2*X - 1
            y1 = 2*Y
            x2 = 2*X
            y2 = 2*Y
        elif(direction==2):
            # This means it went from S-S
            x1 = 2*X + 1
            y1 = 2*Y - 1
            x2 = 2*X + 1
            y2 = 2*Y
        elif(direction==3):        
            # This means it went from W-W
            x1 = 2*X + 1
            y1 = 2*Y + 1
            x2 = 2*X + 2
            y2 = 2*Y + 1
        self.waypoints[self.wpnt_ind][0] = y1 # row
        self.waypoints[self.wpnt_ind][1] = x1 # col
        self.wpnt_ind+=1
        self.waypoints[self.wpnt_ind][0] = y2 # row
        self.waypoints[self.wpnt_ind][1] = x2 # col
        self.wpnt_ind+=1
    
    def right_turn_wpnts(self,arrow):
        # Add three waypoints
        direction = arrow.direction
        end_node = arrow.end_node
        X = end_node.coord_x
        Y = end_node.coord_y
        if(direction==0):
            # This means it went from W-N
            x1 = 2*X
            y1 = 2*Y + 3
            x2 = 2*X 
            y2 = 2*Y + 2
            x3 = 2*X
            y3 = 2*Y + 1
        elif(direction==1):
            # This means it went from N-E
            x1 = 2*X - 2
            y1 = 2*Y
            x2 = 2*X - 1
            y2 = 2*Y
            x3 = 2*X
            y3 = 2*Y
        elif(direction==2):
            # This means it went from E-S
            x1 = 2*X + 1
            y1 = 2*Y - 2
            x2 = 2*X + 1
            y2 = 2*Y - 1
            x3 = 2*X + 1
            y3 = 2*Y
        elif(direction==3):        
            # This means it went from S-W
            x1 = 2*X + 3
            y1 = 2*Y + 1
            x2 = 2*X + 2
            y2 = 2*Y + 1
            x3 = 2*X + 1
            y3 = 2*Y + 1
        self.waypoints[self.wpnt_ind][0] = y1 # row
        self.waypoints[self.wpnt_ind][1] = x1 # col
        self.wpnt_ind+=1
        self.waypoints[self.wpnt_ind][0] = y2 # row
        self.waypoints[self.wpnt_ind][1] = x2 # col
        self.wpnt_ind+=1
        self.waypoints[self.wpnt_ind][0] = y3 # row
        self.waypoints[self.wpnt_ind][1] = x3 # col
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
            y1 = 2*Y + 3
            x2 = 2*X 
            y2 = 2*Y + 3
            x3 = 2*X
            y3 = 2*Y + 2
            x4 = 2*X
            y4 = 2*Y + 1
        elif(direction==1):
            # This means it went from W-E
            x1 = 2*X - 2
            y1 = 2*Y + 1
            x2 = 2*X - 2
            y2 = 2*Y
            x3 = 2*X - 1
            y3 = 2*Y
            x4 = 2*X
            y4 = 2*Y
        elif(direction==2):
            # This means it went from N-S
            x1 = 2*X
            y1 = 2*Y - 2
            x2 = 2*X + 1
            y2 = 2*Y - 2
            x3 = 2*X + 1
            y3 = 2*Y - 1
            x4 = 2*X + 1
            y4 = 2*Y
        elif(direction==3):        
            # This means it went from E-W
            x1 = 2*X + 3
            y1 = 2*Y
            x2 = 2*X + 3
            y2 = 2*Y + 1
            x3 = 2*X + 2
            y3 = 2*Y + 1
            x4 = 2*X + 1
            y4 = 2*Y + 1
        self.waypoints[self.wpnt_ind][0] = y1 # row
        self.waypoints[self.wpnt_ind][1] = x1 # col
        self.wpnt_ind+=1
        self.waypoints[self.wpnt_ind][0] = y2 # row
        self.waypoints[self.wpnt_ind][1] = x2 # col
        self.wpnt_ind+=1
        self.waypoints[self.wpnt_ind][0] = y3 # row
        self.waypoints[self.wpnt_ind][1] = x3 # col
        self.wpnt_ind+=1
        self.waypoints[self.wpnt_ind][0] = y4 # row
        self.waypoints[self.wpnt_ind][1] = x4 # col
        self.wpnt_ind+=1
    
    def draw_graph(self,nodes,parents):
        for node in nodes:
            x = (node[1])*2
            y = (self.rows - node[0] - 1)*2
            plt.plot(x,y,".w")
        for i in range(1,len(parents)):
             x0 = (nodes[i][1])*2
             x1 = (nodes[parents[i]][1])*2
             y0 = (self.rows - nodes[i][0] - 1)*2
             y1 = (self.rows - nodes[parents[i]][0] - 1)*2
             plt.plot(np.array([x0,x1]),np.array([y0,y1]),"-w")

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

if __name__ == "__main__":
    # Ensures it prints entire arrays when logging instead of going [1 1 1 ... 2 2 2]
    np.set_printoptions(threshold=np.inf)

## RUN MULTIPLE SIMULATIONS ##

    # # FIXED PARAMETERS #
    # Imp = False
    # maxIter = 10000
    # dcells = 30
    # cc = 0.1
    # rl = 0.0001
    # obs_perc = 20

    # # Number of simulations per case
    # number_of_sims = 10

    # # VARIABLES #
    # # Grid size range 
    # # OLD WAY OF DOING IT- INDENTED
    #     # max_size = 98
    #     # min_size = 98
    #     # step_size = 10
    #     # sizes = np.array(np.arange(min_size/step_size, max_size /
    #     #                            step_size + 1), dtype=int)*step_size
    # sizes = np.array([58])
    # rows = sizes
    # cols = sizes

    # # Number of robots range
    # # OLD WAY OF DOING IT - INDENTED
    #     # min_robots = 2
    #     # max_robots = 6
    #     # step_robots = 1
    #     # robots = np.array(np.arange(min_robots/step_robots,max_robots/step_robots+1),dtype=int)*step_robots
    # robots = np.array([2, 8, 14, 20])
    # # CALCULATING TOTAL SIMULATIONS AND PRINTING #
    # sims = number_of_sims*len(rows)*len(cols)*len(robots)
    # print("Number of Sims Total: ", sims)

    # # RUNNING SIMULATIONS #
    # sim_overall = 1
    # file_log = "Logging_003.txt"
    # FL = open(file_log, "a")
    # FL.write("Running Zeng style test with  obstacles\n\n")
    # FL.close()
    # for r in rows:
    #     for c in cols:
    #         for robot in robots:
    #             for sim in range(number_of_sims):
    #                 print("SIMULATION: ", sim_overall)
    #                 sim_overall = sim_overall + 1
    #                 obstacles = int((r*c)*obs_perc/100)
    #                 grid_class = generate_grid(r, c, robot, obstacles)
    #                 grid_class.randomise_robots()
    #                 grid_class.randomise_obs()

    #                 EnvironmentGrid = grid_class.GRID
    #                 print_graph = False

    #                 dp = DARP(EnvironmentGrid, dcells, Imp, file_log, print_graph)
    #                 dp.main_DARP()
    #                 if print_graph == True:
    #                     plt.show()
 
## RUN AN INDIVIDUAL CASE ##
    # FIXED PARAMETERS #
    Imp = False
    maxIter = 10000
    obs_perc = 0

    rows = 2
    cols = 9
    n_r = 3
    dcells = int(rows*cols/10)+1
   
    print_graphs = True

    # RUNNING SIMULATION #
    file_log = "Logging_004.txt"

    obstacles = int((rows*cols)*obs_perc/100)
    grid_class = generate_grid(rows, cols, n_r, obstacles)
    grid_class.randomise_robots()
    grid_class.randomise_obs()

    EnvironmentGrid = grid_class.GRID
    
    #  Call this to do directory management and recompile Java files - better to keep separate for when running multiple sims
    algorithm_start(recompile=True)
    # Call this to run DARP and MST
    RA = Run_Algorithm(EnvironmentGrid, dcells, Imp, file_log, print_graphs)
    RA.main()

    if print_graphs == True:
        plt.show()

   
