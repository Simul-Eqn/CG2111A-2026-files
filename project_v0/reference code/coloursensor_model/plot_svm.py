# Source - https://stackoverflow.com/a
# Posted by Matt Hancock, modified by community. See post 'Timeline' for change history
# Retrieved 2025-11-11, License - CC BY-SA 4.0

import pickle as pkl 
import numpy as np
import pandas as pd 
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from sklearn.svm import SVC


df = pd.read_csv('values.csv')
X,Y = df[['R', 'G', 'B']].values, df['COLOUR'].values 

cmap = {
    'MY_RED':'red',
    'MY_GREEN':'green',
    'MY_PINK':'pink',
    'MY_ORANGE':'orange',
    'MY_WHITE':'black',
    'MY_BLUE':'blue',
}
cols = ['MY_BLUE',
   'MY_GREEN',
   'MY_ORANGE',
   'MY_PINK',
   'MY_RED',
   'MY_WHITE']

with open('svm_model.pkl', 'rb') as fin:
    svc = pkl.load(fin) 


# The equation of the separating plane is given by all x in R^3 such that:
# np.dot(svc.coef_[0], x) + b = 0. We should solve for the last coordinate
# to plot the plane in terms of x and y.
def z_fn(x, y, cls1, cls2): 
    return (-(svc.intercept_[cls1]-svc.intercept_[cls2])
            -(svc.coef_[cls1][0]-svc.coef_[cls2][0])*x
            -(svc.coef_[cls1][1]-svc.coef_[cls2][1])*y) / (svc.coef_[cls1][2]-svc.coef_[cls2][2])

tmp = np.linspace(0,300,1501)
x,y = np.meshgrid(tmp,tmp)

# Plot stuff.
fig = plt.figure()
ax  = fig.add_subplot(111, projection='3d')

# # Plot only the selected plane
ax.plot_surface(x, y, z_fn(x,y, 2, 4), color='black', alpha=0.5)

for cls in [2,4]: 
    #ax.plot_surface(x, y, z_fn(x,y, cls), color=cmap[cols[cls]])
    ax.scatter3D(X[Y==cols[cls],0], X[Y==cols[cls],1], X[Y==cols[cls],2], c=cmap[cols[cls]], alpha=0.9)


#ax.plot3D(X[Y==0,0], X[Y==0,1], X[Y==0,2],c='tab:orange')
#ax.plot3D(X[Y==1,0], X[Y==1,1], X[Y==1,2],c='tab:red')
plt.show()
