import DARP_Python_Main as MAIN
import matplotlib.pyplot as plt
import numpy as np
import math
import random as rand

class refuelling:
    def __init__(self,hor,vert):
        # Divide Environment Into Large Nodes
        self.rows = math.ceil(vert/(MAIN.DISC_V*2))
        self.cols = math.ceil(hor/(MAIN.DISC_H*2))
        self.GRID = np.zeros([self.rows, self.cols], dtype=int)
        self.possible_indexes = np.argwhere(self.GRID == 0)
        np.random.shuffle(self.possible_indexes)
    def set_robots_rip(self,n_r,coords):
        # input large cell coordinates
        self.n_r = n_r
        self.rip = coords
        self.rip_sml = np.zeros([len(coords),2],dtype=int)
        self.rip_cont = np.zeros([len(coords),2],dtype=float)
        for r in range(self.n_r):
            self.rip_sml[r][0] = self.rip[r][0]*2
            self.rip_sml[r][1] = self.rip[r][1]*2
            self.rip_cont[r][0] = (self.rip_sml[r][0]+0.5)*MAIN.DISC_V
            self.rip_cont[r][1] = (self.rip_sml[r][1]+0.5)*MAIN.DISC_H
            self.GRID[self.rip[r][0]][self.rip[r][1]] = 2
    def set_obs_rip(self,obs_coords):
        for obs in obs_coords:
            self.GRID[obs[0]][obs[1]] = 1
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
    def determine_refuels(self,n_r):
        cells_per_robot = ( self.rows*self.cols*4 - self.obs*4  ) / n_r
        # calculate path lengths predicted
        dist = max(MAIN.ARC_L,MAIN.DISC_H,MAIN.DISC_V) * cells_per_robot # distance each robot needs should fly
        time = dist / MAIN.VEL # time each robot needs to fly
        flights_req = time / MAIN.FLIGHT_TIME # fights required per robot
        refuels = math.ceil(flights_req) - 1
        return(refuels)
    def possible_robots(self,n_r):
        # Note: self.n_r needs to be determined before this function can be called
        self.possible_robots_list = list()
        self.cardinal_dir = [[1,0],[-1,0],[0,1],[0,-1]]
        okay = False
        if self.n_r <= 4:
            self.moves = np.array([[-1,0],[0,1],[1,0],[0,-1]])
            self.pos = np.array([[0,1],[1,1],[1,0],[0,0]]) # Small block position shift
        elif (self.n_r > 4)and(self.n_r <= 8):
            self.moves = np.array([[-2,0],[0,2],[2,0],[0,-2],[-2,2],[2,2],[2,-2],[-2,-2]])
            self.pos = np.array([[0,1],[1,1],[1,0],[0,0],[0,1],[1,1],[1,0],[0,0]])
        elif (self.n_r > 8)and(self.n_r <= 12):
            self.moves = np.array([[-3,-3],[-3,1],[-1,3],[3,3],[3,-1],[1,-3],[-3,-1],[-3,3],[1,3],[3,1],[3,-3],[-1,-3]])
            self.pos = np.array([[0,0],[0,1],[0,1],[1,1],[1,0],[1,0],[0,0],[0,1],[1,1],[1,1],[1,0],[0,0]])
        elif (self.n_r > 12)and(self.n_r <= 16):
            self.moves = np.array([[-4,-4],[-4,0],[-4,4],[0,4],[4,4],[4,0],[4,-4],[0,-4],[-4,-2],[-4,2],[-2,4],[2,4],[4,2],[4,-2],[2,-4],[-2,-4]])
            self.pos = np.array([[0,0],[0,0],[0,1],[0,1],[1,1],[1,1],[1,0],[1,0],[0,0],[0,1],[0,1],[1,1],[1,1],[1,0],[1,0],[0,0]])
        for r in range(self.rows):
            for c in range(self.cols):
                if self.GRID[r][c] != 1:
                    # Check relevant robot positions
                    okay = True
                    for n in range(self.n_r):
                        # Check if robot position have obstacles in them
                        r_n = r + self.moves[n][0]
                        c_n = c + self.moves[n][1]
                        if(r_n<self.rows)and(r_n>=0)and(c_n<self.cols)and(c_n>=0):
                            if self.GRID[r_n][c_n] == 1:
                                okay = False
                        else:
                            okay = False
                        # Check for obstacles in vicinity of valid robot location - make sure at least one of the cardinal sides is FREE
                        ob_count = 4
                        if okay == True:
                            for m in self.cardinal_dir:
                                r_o = r_n + m[0]
                                c_o = c_n + m[1]
                                if(r_o<self.rows)and(r_o>=0)and(c_o<self.cols)and(c_o>=0):
                                    if self.GRID[r_o][c_o] == 1:
                                        ob_count-=1
                                else:
                                    ob_count-=1
                                if ob_count == 0:
                                    okay = False   
                if okay == True:
                    self.possible_robots_list.append([r,c])
    def refuel_randomise_start(self,n_r):
        try:
            # Conservatively calculate the number of refuels needed
            refuels = self.determine_refuels(n_r)
            self.n_r = n_r * (refuels+1) # equivalent number of robots given the number of refuels
            # Print equivalent robots and exit if more than 16 (possible robots can only do up to 16 for now)
            print("Robots: ", n_r," Refuels: ", refuels," Equivalent Robots: ", self.n_r)
            if self.n_r > 16:
                print("Number of equivalent robots are beyond algorithm capability")
                return(False)
            # Calculate possible starting positions
            self.possible_robots(n_r)
            if len(self.possible_robots_list) == 0:
                print("WARING: No valid starting location found...")
            # Randomly choose a valid starting position and set rip
            self.rip = np.zeros([self.n_r,2],dtype=int)
            self.rip_sml = np.zeros([self.n_r,2],dtype=int)
            self.rip_cont = np.zeros([self.n_r,2],dtype=float)
            start = self.possible_robots_list[ rand.randint(0,len(self.possible_robots_list)) ]
            for r in range(self.n_r):
                move = self.moves[r]
                self.rip[r][0] = move[0] + start[0]
                self.rip[r][1] = move[1] + start[1]
                pos = self.pos[r]
                self.rip_sml[r][0] = self.rip[r][0]*2 + pos[0]
                self.rip_sml[r][1] = self.rip[r][1]*2 + pos[1]
                self.rip_cont[r][0] = (self.rip_sml[r][0]+0.5)*MAIN.DISC_V
                self.rip_cont[r][1] = (self.rip_sml[r][1]+0.5)*MAIN.DISC_H
                self.GRID[self.rip[r][0]][self.rip[r][1]] = 2
            # Make centre of robot formation (the landing and take off zone) an obstacle
            self.set_obs_rip([start])
            return(True)
        except:
            print("An error occurred in the refuelling protocol...")
            return(False)
                  
# Ensures it prints entire arrays when logging instead of going [1 1 1 ... 2 2 2]
np.set_printoptions(threshold=np.inf)

# What graphs should it print
MAIN.PRINT_DARP = True
MAIN.PRINT_TREE = False
MAIN.PRINT_PATH = True
MAIN.PRINT_CIRCLE_CENTRES = False

# Establish Environment Size - Chooses max horizontal and vertical dimensions and create rectangle
horizontal = 3000.0 # m
vertical = 3000.0 # m

# Establish Small Node size
GG = refuelling(horizontal,vertical)

# Coordinates from top left (vert,hor)
n_r = 2
obs_perc = 0

GG.randomise_obs(obs_perc)
GG_Success = GG.refuel_randomise_start(n_r)

if GG_Success == True:
    # Other parameters
    distance_measure = 0 # 0, 1, 2 - Euclidean, Manhattan, GeodisicManhattan
    Imp = False
    maxIter = 10000

    rows = GG.rows
    cols = GG.cols
    obs = GG.obs # Note this is before removal of enclosed space, which can increase the number of robots
    n_r_equivalent = GG.n_r
    dcells = math.ceil( ((rows*cols-obs)/n_r_equivalent)*0.3 ) # discrepancy of X% allowed

    print_graphs = True

    # RUNNING SIMULATION #
    file_log = "MAIN_LOGGING.txt"
    target_log = "TARGET_LOG.txt"
    EnvironmentGrid = GG.GRID

    #  Call this to do directory management and recompile Java files - better to keep separate for when running multiple sims
    MAIN.algorithm_start(recompile=True)

    # Call this to run DARP and MST
    RA = MAIN.Run_Algorithm(EnvironmentGrid, GG.rip, dcells, Imp, print_graphs,dist_meas=distance_measure,log_active=False,log_filename=file_log,target_filename=target_log,target_active=True)
    RA.set_continuous(GG.rip_sml,GG.rip_cont)
    RA.main()

    if print_graphs == True:
        plt.show()
