import numpy as np
import numpy.matlib
import matplotlib.pyplot as plt
from scipy.io import loadmat

fig = plt.figure()
ax1=fig.add_subplot(1,1,1)


file = "data_walk_pipms_TAMP.mat"
ugrid=loadmat(file)['ugrid']
xgrid=loadmat(file)['xgrid']
goalset=loadmat(file)['goalset']
guardset=loadmat(file)['guardset']
initset=loadmat(file)['initset']
leastctlr1=loadmat(file)['leastctlr1']
leastctlr2=loadmat(file)['leastctlr2']

x0 = ([0, 0.4])

dx = xgrid -np.matlib.repmat(x0,xgrid.shape[0],1)
dxx = np.multiply(dx[:,0],dx[:,0]) + np.multiply(dx[:,1],dx[:,1])

ind = numpy.where(dxx == numpy.amin(dxx))[0]
print(ind)
ax1.plot(ugrid)


plt.show()