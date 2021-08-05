import matplotlib.pyplot as plt
import matplotlib.patches as pat
import numpy as np

fig,ax = plt.subplots(figsize=[8,8])

e1 = pat.Arc([0.5,0.5],0.2,0.2,angle=0.0,theta1=0.0,theta2=90.0,ls='-.',color='b')
ax.add_patch(e1)
e2 = pat.Arc([0.5,0.5],0.2,0.2,angle=90.0,theta1=0.0,theta2=90.0,ls='-.', color='g')
ax.add_patch(e2)
e3 = pat.Arc([0.5,0.5],0.2,0.2,angle=180.0,theta1=0.0,theta2=90.0,ls='-.', color='r')
ax.add_patch(e3)
e4 = pat.Arc([0.5,0.5],0.2,0.2,angle=270.0,theta1=0.0,theta2=90.0,ls='-.', color='k')
ax.add_patch(e4)
plt.show()