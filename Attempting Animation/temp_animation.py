# import numpy as np
# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation
# import matplotlib.animation as animation

# fig = plt.figure()
# ax = plt.axes(xlim=(0,4),ylim=(-2,2))
# line, = ax.plot([],[],lw=3)

# def init():
#     line.set_data([],[])
#     return line,
# def animate(i):
#     x = np.linspace(0,4,1000)
#     y = np.sin(2*np.pi * (x-0.01*i))
#     line.set_data(x,y)
#     return line,

# anim = FuncAnimation(fig,animate,init_func=init,frames=200,interval=20,blit=True)

# FFMpegWriter = animation.writers['ffmpeg']
# writer = animation.FFMpegWriter()

# anim.save("sinewave.mp4",writer=writer)

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math

fig, ax = plt.subplots()
x = np.linspace(0,10,100)
y = (100 - x**2)**0.5
l0 = plt.plot(x,y,'r-')

l1 = plt.plot([-5,0],[5,5],'b-')
x2 = np.linspace(0,5,100)
y2 = (25 - x2**2)**0.5
l2 = plt.plot(x2,y2,'b-')
l3 = plt.plot([5,5],[0,-5],'b-')
# ax.set_xticks()
# ax.set_yticks()
ax = plt.axis([-10,10,-10,10])

redDot, = plt.plot([0], [np.sqrt(100)], 'ro')
blueDot, = plt.plot([0], [np.sqrt(25)], 'bo')

class A:
    def __init__(self):
        self.vals = np.linspace(0,-5,10*5)
        self.temp = 0
    def animate(self,i):
        if((i<=10) and (i>=0)):
            redDot.set_data(i, (100-i**2)**0.5)
        if(i<0):
            blueDot.set_data(5,i)
        elif((i<=5)and (i>=0)):
            blueDot.set_data(i,(25 - i**2)**0.5)
        elif(i>5):
            blueDot.set_data(5,self.vals[self.temp])
            self.temp+=1
        return [[redDot,] , [blueDot,]]

# def animate2(i):
#     blueDot.set_data(i,(25 - i**2)**0.5)
#     return blueDot,

# create animation using the animate() function
animate_class = A()
myAnimation = animation.FuncAnimation(fig, animate_class.animate, frames=np.arange(-5,10.1,0.1))
# myAnimation2 = animation.FuncAnimation(fig, animate2,frames=np.arange(0,6,1))

myAnimation.save('circle.mp4', fps=30, extra_args=['-vcodec', 'libx264'])