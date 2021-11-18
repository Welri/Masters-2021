import matplotlib.pyplot as plt
import numpy as np
import math

class path_obj:
    def __init__(self,Ccd,Cca,Pd,Pa,Hsl,Lsl,Hf,PathLen):
        self.Ccd = Ccd
        self.Cca = Cca
        self.Pd = Pd
        self.Pa = Pa
        self.Hsl = Hsl
        self.Lsl = Lsl
        self.Hf = Hf
        self.PathLen = PathLen

class path_planner:
    def __init__(self,wpnt_start,wpnt_end,circle_R):
        self.R = circle_R
        self.Head_start =wpnt_start[1]%(2*math.pi)
        self.Head_end = wpnt_end[1]%(2*math.pi)
        self.PS = wpnt_start[0]
        self.PE = wpnt_end[0]
        self.HS = np.array([math.cos(self.Head_start),math.sin(self.Head_start)])
        self.HE = np.array([math.cos(self.Head_end),math.sin(self.Head_end)])

        self.CW90 = np.array([[0,-1],[1,0]])
        self.ACW90 = np.array([[0,1],[-1,0]])
    def shortest_path(self):
        # RR - Initial right turn, then right hand circuit
        SCcR = self.PS + np.dot(self.ACW90,self.HS)*self.R # Centre of initial right circle
        ECcR = self.PE + np.dot(self.ACW90,self.HE)*self.R # Centre of final right circle
        E_S = ECcR - SCcR # Vector from right circle to final right circle
        m2RR = np.linalg.norm(E_S) # Length between circle centres
        HcentresRR = (E_S)/m2RR # Unit vector from right circle centre to final right circle centre
        PdRR_SCcR = np.dot(self.CW90,HcentresRR)*self.R # Vector from centre start right circle to departure point    
        PdRR = SCcR + PdRR_SCcR # Departure point from right circle	               
        PaRR = PdRR + E_S # Arrival point after right circle             
        
        HdRR = (math.atan2(HcentresRR[1],HcentresRR[0]))%(2*math.pi)
        PathLenRR = self.R*((self.Head_start - HdRR)%(2*math.pi) + (HdRR - self.Head_end)%(2*math.pi)) + m2RR

        # Store
        self.paths = np.empty(4,dtype=np.object)
        self.paths[0] = path_obj(SCcR,ECcR,PdRR,PaRR,HcentresRR,m2RR,self.HE,PathLenRR)
        # self.Ccd = SCcR
        # self.Cca = ECcR
        # self.Pd = PdRR
        # self.Pa = PaRR
        # self.Hsl = HcentresRR
        # self.Lsl = m2RR
        # self.Hf = self.HE
        # self.shortestPathLen = PathLenRR

        # RL - Initial Right Turn, then Left Hand Circuit
        ECcL = self.PE + np.dot(self.CW90,self.HE)*self.R                    # Centre of final left circle
        E_S = ECcL - SCcR                           # Vector from right circle to final left circle 
        m2RL = np.linalg.norm(E_S) 	                # Length between circle centres
        mRL = m2RL/2                                # Half length between circle centres
        if (mRL > self.R):                          # Continue only if length between circle centres is greater than 2*self.R
            HcentresRL = (E_S)/m2RL                 # Unit vector from right circle centre to final left circle centre
            sRL = math.sqrt(mRL**2 - self.R**2)     # Half length of straight path between departure and arrival points
            PcRL = (SCcR + ECcL)/2    	            # Midpoint between circles
            ROTRL = np.array([[sRL, self.R],[-self.R, sRL]])/mRL 	# Rotation matrix. s/m = cos(theta_cross), r/m=sin(theta_cross)
            HdiagRL = np.dot(ROTRL,HcentresRL)              # Heading on path between circles
            PdRL = PcRL - HdiagRL*sRL               # Departure point from right circle
            PaRL = PcRL + HdiagRL*sRL               # Arrival point at left circle
            # Calculate path length
            HdRL = (math.atan2(HdiagRL[1],HdiagRL[0]))%(2*math.pi)
            PathLenRL = self.R*((self.Head_start - HdRL)%(2*math.pi) + (self.Head_end - HdRL)%(2*math.pi)) + 2*sRL
            # Store
            self.paths[1] = path_obj(SCcR,ECcL,PdRL,PaRL,HdiagRL,2*sRL,self.HE,PathLenRL)
            # if (PathLenRL < self.shortestPathLen):
            #     self.Ccd = SCcR
            #     self.Cca = ECcL
            #     self.Pd = PdRL
            #     self.Pa = PaRL
            #     self.Hsl = HdiagRL
            #     self.Lsl = 2*sRL
            #     self.Hf = self.HE
            #     self.shortestPathLen = PathLenRL
        else:
            PathLenRL = np.inf
        
        # LL - Initial Left Turn, then Left Hand Circuit
        SCcL = self.PS + np.dot(self.CW90,self.HS)*self.R                   # Centre of initial left circle
        ECcL = self.PE + np.dot(self.CW90,self.HE)*self.R                    # Centre of final left circle
        E_S = ECcL - SCcL                    # Vector from initial left circle to final left circle 
        m2LL = np.linalg.norm(E_S)        # Length between circle centres
        HcentresLL = (E_S)/m2LL              # Unit vector (heading) from Ccl to Ca
        PdLL_SCcL = np.dot(self.ACW90,HcentresLL)*self.R          # Vector from centre left circle to departure point
        PdLL = SCcL + PdLL_SCcL          	    # Departure point from left circle
        PaLL = PdLL + E_S                   # Arrival point after left circle
        # Calculate path length
        HdLL = (math.atan2(HcentresLL[1],HcentresLL[0]))%(2*math.pi)
        PathLenLL = self.R*((HdLL - self.Head_start)%(2*math.pi) + (self.Head_end - HdLL)%(2*math.pi)) + m2LL
        # Store
        self.paths[2] = path_obj(SCcL,ECcL,PdLL,PaLL,HcentresLL,m2LL,self.HE,PathLenLL)
        # if (PathLenLL < self.shortestPathLen):
        #     self.Ccd = SCcL
        #     self.Cca = ECcL
        #     self.Pd = PdLL
        #     self.Pa = PaLL
        #     self.Hsl = HcentresLL
        #     self.Lsl = m2LL
        #     self.Hf = self.HE
        #     self.shortestPathLen = PathLenLL

        # LR - Initial Left Turn, then Right Hand Circuit
        ECcR = self.PE + np.dot(self.ACW90,self.HE)*self.R         		    # Centre of right circle
        E_S = ECcR - SCcL	                # Vector from left circle to final circle 
        m2LR = np.linalg.norm(E_S)        # Length between circle centres
        mLR = m2LR/2                               # Half length between circle centres
        if (mLR > self.R):                              # Continue only if length between circle centres is greater than 2*Rad
            HcentresLR = (E_S)/m2LR      	# Unit vector from right circle centre to final left circle center
            sLR = math.sqrt(mLR**2 - self.R**2)       		# Half length of straight path between departure and arrival points
            PcLR = (SCcL + ECcR)/2             	# Midpoint between circles
            ROTLR = np.array([[sLR, -self.R], [self.R, sLR]])/mLR  	# Rotation matrix. s/m=cos(theta_cross), r/m=sin(theta_cross)
            HdiagLR = np.dot(ROTLR,HcentresLR)             # Heading on path between circles
            PdLR = PcLR - HdiagLR*sLR             	# Departure point from left circle
            PaLR = PcLR + HdiagLR*sLR		        # Arrival point after left circle
            # Calculate path length
            HdLR = (math.atan2(HdiagLR[1],HdiagLR[0]))%(2*math.pi)
            PathLenLR = self.R*((HdLR - self.Head_start)%(2*math.pi) + (HdLR - self.Head_end)%(2*math.pi)) + 2*sLR
            # Store
            self.paths[3] = path_obj(SCcL,ECcR,PdLR,PaLR,HdiagLR,2*sLR,self.HE,PathLenLR)
            # if (PathLenLR < self.shortestPathLen):
            #     self.Ccd = SCcL
            #     self.Cca = ECcR
            #     self.Pd = PdLR
            #     self.Pa = PaLR
            #     self.Hsl = HdiagLR
            #     self.Lsl = 2*sLR
            #     self.Hf = self.HE
            #     self.shortestPathLen = PathLenLR
        else:
            PathLenLR = np.inf

        path = self.paths[0]
        self.shortest_path = path
        for i in range(1,4):
            path = self.paths[i]
            if(path.PathLen < self.shortest_path.PathLen):
                self.shortest_path=path
    def plot_shortest_path(self):
        AR = 1/1 # (y/x)
        xaxis = 700
        yaxis = AR*xaxis
        plt.figure(figsize=[xaxis/100,yaxis/100])
        plt.xlabel('East')
        plt.ylabel('North')
        plt.title('Shortest Path')
        plt.grid()
        plt.axis([-xaxis/2,xaxis,-yaxis/2,yaxis])
        xcirc=self.R*np.cos(np.linspace(0,2*math.pi,100)) 
        ycirc=self.R*np.sin(np.linspace(0,2*math.pi,100))
        HS = np.array([math.cos(self.Head_start), math.sin(self.Head_start)])
        HE = np.array([math.cos(self.Head_end), math.sin(self.Head_end)])
        plt.plot(self.PS[0]+np.array([0, -1])*HS[0]*2*self.R, self.PS[1]+np.array([0, -1])*HS[1]*2*self.R,'C0')   # Starting direction vector
        plt.plot(self.PE[0]+np.array([0, 1])*HE[0]*2*self.R, self.PE[1]+np.array([0, 1])*HE[1]*2*self.R,'C0')     # Ending direction vector
        plt.text(PS[0]-2.3*self.R*HS[0],PS[1]-2.3*self.R*HS[1],'Start')          # Text on graph
        plt.text(PE[0]+2.3*self.R*HE[0],PE[1]+2.3*self.R*HE[1],'End')            # Text on graph
        plt.plot(xcirc+self.shortest_path.Ccd[0],ycirc+self.shortest_path.Ccd[1],'C2')                            # Departure circle
        plt.plot(xcirc+self.shortest_path.Cca[0],ycirc+self.shortest_path.Cca[1],'C2')                            # Arrival circle
        plt.plot([self.shortest_path.Pa[0], self.shortest_path.Pd[0]],[self.shortest_path.Pa[1],self.shortest_path.Pd[1]],'C0')                          # Straight path
    def plot_paths(self,separate_plots=False):
        AR = 1/1 # (y/x)
        xaxis = 700
        yaxis = AR*xaxis
        if(separate_plots==False):
            plt.figure(figsize=[xaxis/100,yaxis/100])
            plt.title('Four Possible Paths')
            plt.xlabel('East')
            plt.ylabel('North')
            plt.grid()
            plt.axis([-xaxis/2,xaxis,-yaxis/2,yaxis])
        xcirc=self.R*np.cos(np.linspace(0,2*math.pi,100)) 
        ycirc=self.R*np.sin(np.linspace(0,2*math.pi,100))
        HS = np.array([math.cos(self.Head_start), math.sin(self.Head_start)])
        HE = np.array([math.cos(self.Head_end), math.sin(self.Head_end)])
        if(separate_plots==False):
            plt.plot(self.PS[0]+np.array([0, -1])*HS[0]*2*self.R, self.PS[1]+np.array([0, -1])*HS[1]*2*self.R,'C0')   # Starting direction vector
            plt.plot(self.PE[0]+np.array([0, 1])*HE[0]*2*self.R, self.PE[1]+np.array([0, 1])*HE[1]*2*self.R,'C0')     # Ending direction vector
            plt.text(PS[0]-1.6*self.R*HS[0],PS[1]-1.4*self.R*HS[1],'Start')          # Text on graph
            plt.text(PE[0]+2.3*self.R*HE[0],PE[1]+2.3*self.R*HE[1],'End')            # Text on graph
        if(separate_plots==True):
            titles = ['RR Plot','RL Plot','LL Plot','LR Plot']
        for i in range(4):
            if(separate_plots==True):
                plt.figure(figsize=[xaxis/100,yaxis/100])
                plt.title(titles[i])
                plt.xlabel('East')
                plt.ylabel('North')
                plt.grid()
                plt.axis([-xaxis/2,xaxis,-yaxis/2,yaxis])
                plt.plot(self.PS[0]+np.array([0, -1])*HS[0]*2*self.R, self.PS[1]+np.array([0, -1])*HS[1]*2*self.R,'C0')   # Starting direction vector
                plt.plot(self.PE[0]+np.array([0, 1])*HE[0]*2*self.R, self.PE[1]+np.array([0, 1])*HE[1]*2*self.R,'C0')     # Ending direction vector
                plt.text(PS[0]-1.6*self.R*HS[0],PS[1]-1.4*self.R*HS[1],'Start')          # Text on graph
                plt.text(PE[0]+2.3*self.R*HE[0],PE[1]+2.3*self.R*HE[1],'End')            # Text on graph
            plt.plot(xcirc+self.paths[i].Ccd[0],ycirc+self.paths[i].Ccd[1],'C2')                            # Departure circle
            plt.plot(xcirc+self.paths[i].Cca[0],ycirc+self.paths[i].Cca[1],'C2')                            # Arrival circle
            plt.plot([self.paths[i].Pa[0], self.paths[i].Pd[0]],[self.paths[i].Pa[1],self.paths[i].Pd[1]],'C0') 



V = 30
Phi_max = 26 # max bank angle
r_min = V*V/9.81*np.tan(Phi_max) 
r_min = 100

# Start and End coordinates
PS = np.array([0,0])
PE = np.array([500,500])
# Start and End headings
Head_start = 120*math.pi/180
Head_end = 320*math.pi/180
start = [PS,Head_start]
end = [PE,Head_end]

PP = path_planner(start,end,r_min)
PP.shortest_path()
PP.plot_shortest_path()
# PP.plot_paths(separate_plots=True)
plt.show()
