import sys, os, glob
import argparse

import PIL
import cv2
import numpy as np
import matplotlib.pyplot as plt
from skfmm import distance, travel_time
from vis2d import vis2d

from matlab_utils import keyboard, tic, toc, imagesc

# global colormap
plt.rcParams['image.cmap'] = 'gray'


def s2x(s, h, w, dx):
    # convert from index to x,y subscript
    s = np.array(s)
    i = np.expand_dims(s // w, axis=-1)
    j = np.expand_dims(s % w, axis=-1)

    x = np.concatenate((i, j), axis=1) * dx

    return x

def x2s(x, h, w, dx):
    # convert from x,y subscript to index   
    temp = np.round(x/dx)
    s = temp[:,0] * w + temp[:,1]
    return s




def im2double(im):
    min_val = np.min(1*im.ravel())
    max_val = np.max(1*im.ravel())
    out = (im.astype('float') - min_val) / (max_val - min_val)
    return out


def computeVisibility(phi, psi, x0, dx):
    # assumes x0 is [n,3] matrix, each row is an observing location
    psi_current = vis2d(phi, np.round(x0[-1,:]/dx).astype(int))
    #psi_current = distance(psi_current,dx)
    psi = np.maximum(psi, psi_current)

    return psi, psi_current


def computeGain(phi, psi, x0, x, y, viewRadius = np.inf, isSurveillance = True):

    dx = float(x[0,1] - x[0,0])
    eps = 2*dx

    finished = False
    [m,n] = phi.shape[:2]

    psi,_ = computeVisibility(phi, psi, x0, dx)

    E = 0 * phi
    for i in range(0,m):
        for j in range(0,n):
                xc = np.array([i,j])*dx
                new_psi,_ = computeVisibility(phi, psi, np.concatenate((x0,[xc]),axis=0), dx)
                gain = 1*(psi < 0 ) - 1*(new_psi < 0 )
                gain = gain.sum() * dx * dx 
                E[i,j] = gain

    if not isSurveillance:
        E = E*(psi>0)  # only visibile regions should have gain

    #E = E/(E.max() + 1e-12)  # we have to normalize otherwise nothing is seen


    [xr,xc] = np.where(E==E.max())
    ind = np.random.randint(len(xc))
    xr = xr[ind]
    xc = xc[ind]
    x0 = np.concatenate((x0, np.array([[y[xr,xc], x[xr,xc]]])), axis=0)
  
    return x0, psi, E, finished


def getRandomPositions(mask, num = 1):

        idx = np.where(mask>0)
        numPositions = idx[0].shape[0]
        randIdx = np.random.choice(numPositions, num, replace=False)

        return [ idx[0][randIdx], idx[1][randIdx] ]


class env:
    def __init__(self, map_file = None, vP = 3, vE = 1, windowSize = 5):
        self.vP = vP 
        self.vE = vE
      
        self.dt = None  # if using fixed time intervals
        self.knownEnv = True # if env is known

        # setup the env 
        im = im2double(cv2.imread(map_file,0))
        m = im.shape[0] 
        dx = 1.0/m
        self.m = m
        self.dx = dx
        x = np.linspace(0,m-1,m) * dx
        [self.x, self.y] = np.meshgrid(x,x)
        self.phi = distance( (2*im-1)*dx, dx)

        # speeds for P and E
        self.speed = 1.0 * (self.phi > 0)  # assume uniform speed

        # initialize paramters for static game
        self.occTime = np.zeros((m,m))
        self.uP = np.ma.core.MaskedArray(np.zeros((m,m)))
        self.uE = np.ma.core.MaskedArray(np.zeros((m,m)))
        self.tStar = 0
        self.R = np.ma.core.MaskedArray(np.zeros((m,m)))

        # initialize parameters for sliding visibility
        self.slidingVis = np.zeros((m,m))  # num of sensors visible from each pixel
        self.totalVis = np.zeros((m,m))  # keep track the whole time
        self.psiQueue = []                 # queue of maxsize windowSize
        self.windowSize = windowSize
    
        #self.iP_f = 0
        #self.jP_f = 0

        # path variables
        self.pathP = None
        self.pathE = None
        self.lastPathLengthP = 0
        self.lastPathLengthE = 0


    def position_to_sub(self, xP, xE):
        iP = int(xP[0,0]/self.dx)
        jP = int(xP[0,1]/self.dx)
        iE = int(xE[0,0]/self.dx)
        jE = int(xE[0,1]/self.dx)
    
        return [iP, jP, iE, jE]


    def pushVis(self, newPsi):
        # add in new visibility
        self.totalVis = self.totalVis + 1*(newPsi>0)
        self.slidingVis = self.slidingVis + 1*(newPsi>0)
        self.psiQueue.insert(0, newPsi) 

        # update the queue
        if len(self.psiQueue) > self.windowSize:
            # subtract the last visibility
            self.slidingVis = self.slidingVis - 1*(self.psiQueue.pop()>0)

        temp = -1e12 + 0 * self.slidingVis
        
        for i in range(min(len(self.psiQueue), self.windowSize-1)):
            temp = np.maximum(temp, self.psiQueue[i]) 
       
        return temp 


    def compute_action_set(self, s, v):
    # gets list of actions from a state s given velocity v

        m = self.m
        x = s2x([s], m, m, 1)
        i = int(x[-1,0])
        j = int(x[-1,1])
        print('s = {}\n m = {}\nv = {}\nx = {}\ni = {}\nj = {}'.format(s,self.m,v,x,i,j))
        wP = np.ones((m,m)) 
        wP[i,j] = 0
        wP = travel_time(wP, v * self.speed, dx = 1)
       
        mask =  (wP.data<=1)*1*(self.phi>0) 
        # this is if we want to convert to states
        #X,Y = np.where(mask)
        #actions = Y * m + X

        return mask

    def compute_reach_gain_path(self, sP, sE):
        #[iP, jP, iE, jE] = self.position_to_sub(xP,xE)
      
        m = self.m
        dx = self.dx
        speed = self.speed
        phi = self.phi

        xP = s2x([sP], m, m, dx)
        xE = s2x([sE], m, m, dx)
        psi = self.pushVis(computeVisibility(self.phi, -1e12, xP, dx)[0])
     
        self.pathP = 1*xP
        self.pathE = 1*xE

        #self.plot_reach_gain_path(xP, xE, psi, 0*psi + 1, 'figures/plot_0.png')

        # compute gain
        _, _ , E, _  = computeGain(phi, psi, xP, self.x, self.y, viewRadius = np.inf, isSurveillance = True)

        if not self.knownEnv:
            E = E * (self.totalVis>0) 
        # compute max gain within reachable set 
        self.compute_occ_time( np.expand_dims(xP[-1,:],0),np.expand_dims(xE[-1,:],0), knownEnv = self.knownEnv)

        maximizeWithinReachSet = True
        if maximizeWithinReachSet:

            #border = np.zeros((m,m))
            #border[2:m-2,2:m-2] = 1
            E = E* (1-self.R.mask) 
            [xi,xj] = np.where(E==E.max())
            ind = np.random.randint(len(xi))
            iP = xi[ind]
            jP = xj[ind]
            self.iP = iP
            self.jP = jP
            xP = np.concatenate((xP, np.array([[self.y[iP,jP], self.x[iP,jP]]])), axis=0)

        else:
            # move P towards argmax energy
            # TODO: change xi to iP
            [xi,xj] = np.where(E==E.max())
            ind = np.random.randint(len(xi))
            iP_f = xi[ind]
            jP_f = xj[ind]
            #self.iP_f = iP_f
            #self.jP_f = jP_f


            wP = np.ones((m,m)) 
            wP[iP_f,jP_f] = 0
            wP = travel_time(wP, self.vP * speed, dx = dx)

            # this picks the closest point to iP,jP within reachable set
            #wP = wP + 1000 * (self.uP >= self.tStar)
            #wP = wP - 1000 * (self.uP.data > 0)*1*((self.occTime-self.uP.data)>=0)

            if self.dt:
                wP = wP - 1000 * (self.uP.data > 0)*1*((self.dt-self.uP.data)>=0)
            else:
                wP = wP - 1000 * (self.uP.data > 0)*1*((self.occTime-self.uP.data)>=0)

            [iP, jP] = getRandomPositions( (wP.data == wP.min()) * (self.uP.data > 0)*1*((self.occTime-self.uP.data)>=0) )
            #[iP, jP] = getRandomPositions( wP.data == wP.min() )
            iP = iP[0]
            jP = jP[0]
            xP = np.concatenate((xP, np.array([[self.y[iP,jP], self.x[iP,jP]]])), axis=0)

        # compute random E position within its reach set
  
        if self.dt:
            self.tStar = self.dt
        else:
            self.tStar = self.uP[iP,jP] 
        [iE_f, jE_f] = getRandomPositions( (self.uE.data <= self.tStar) * 1*(phi>0) )
        iE_f = iE_f[0]
        jE_f = jE_f[0]
                 
        xE = np.concatenate((xE, np.array([[self.y[iE_f,jE_f], self.x[iE_f,jE_f]]])), axis=0)
        psi = self.pushVis(computeVisibility(self.phi, -1e12, np.expand_dims(xP[-1,:],0), dx)[0])
 
        [self.pathP, self.lastPathLengthP] = self.update_path(self.pathP, xP, self.uP, self.vP)
        [self.pathE, self.lastPathLengthE] = self.update_path(self.pathE, xE, self.uE, self.vE)

        #self.plot_reach_gain_path(xP, xE, psi, E, 'plot.png')
        
        return self.discretize_path(self.pathP)
        

    def update_path(self, X0, Xf, u, v):
        # u is the travel time from initial position
        # v is velocity
        # naively let's compute the path backwards by picking the closest point within the radius given by mask
        m = self.m
        dx = self.dx
        u = u.data + 1000*u.mask
        path = np.array([Xf[-1]])

        while np.abs(path[-1] - X0[-1]).sum() > dx:
            s = x2s(np.array([path[-1]]), m, m, dx)
            mask = self.compute_action_set(s,v)
            temp = u + 1000*(1-mask)
            [i,j] = getRandomPositions( temp == temp.min() )
            i = i[0]; j = j[0]
            path = np.concatenate((path, np.array([[self.y[i,j], self.x[i,j]]])), axis=0)
        path = path[::-1]  #reverse

        return path, path.shape[0]
                
    def discretize_path(self, path):
    # transform path to discrete version
        m = self.m
        dx = 1.0/m
        sPath = x2s(path, m, m, dx)
        sPath = sPath[np.sort(np.unique(sPath,return_index=True)[1])]
        # hacky way to incorporate speed, keep every self.vP positions
        #sPath = sPath[self.vP::self.vP]
        return sPath

    def plot_reach_gain_path(self, xP, xE, psi, E, name):
        dx = self.dx
        j = min(xP.shape[0], self.windowSize)
        k = min(xP.shape[0], 2)
        kP = self.lastPathLengthP
        kE = self.lastPathLengthE

        print(kP, kE)
        plt.subplot(2,2,1)
        plt.imshow(self.phi>0)
        plt.plot(xE[:,1]/dx,xE[:,0]/dx,'r.')
        plt.plot(xP[:,1]/dx,xP[:,0]/dx,'b.')
        plt.plot(xE[-1,1]/dx,xE[-1,0]/dx,'ro')
        plt.plot(xP[-1,1]/dx,xP[-1,0]/dx,'bs')

        plt.plot(xE[:,1]/dx,xE[:,0]/dx,'r.')
        plt.plot(xP[:,1]/dx,xP[:,0]/dx,'b.')

        plt.plot(self.pathP[:,1]/dx, self.pathP[:,0]/dx, 'b')
        plt.plot(self.pathE[:,1]/dx, self.pathE[:,0]/dx, 'r')

        plt.contour(self.phi,0)
        plt.title('Pursuer (Blue) and Evader (Red)')

        plt.subplot(2,2,2)
        plt.imshow(self.phi>0)
        plt.plot(xE[-1,1]/dx,xE[-1,0]/dx,'ro')
        plt.plot(xP[-1,1]/dx,xP[-1,0]/dx,'bs')
        plt.plot(self.pathE[-kE:,1]/dx,self.pathE[-kE:,0]/dx,'r-')
        plt.plot(self.pathP[-kP:,1]/dx,self.pathP[-kP:,0]/dx,'b-')


        #plt.contour((self.uP.data > 0)*1*((self.occTime-self.uP.data)>=0), 0.5,  colors='blue', linestyles='dashed')
        plt.contour(self.uP.data + 100*self.uP.mask, self.tStar,  colors='blue', linestyles='dashed')
        plt.contour(self.uE, self.tStar,  colors='red', linestyles='dashed')
        plt.contour(self.phi,0)
        plt.title('Reach sets')
        plt.subplot(2,2,3)
        plt.imshow(E)
        plt.plot(self.pathP[-kP:,1]/dx, self.pathP[-kP:,0]/dx, 'b-')
        plt.plot(self.pathP[-1,1]/dx,  self.pathP[-1,0]/dx,'bs')

        plt.contour(self.R.mask*np.ones((self.m,self.m)), 0.5, colors = 'blue') #TODO: hacky npones
        plt.title('Gain Function and Ideal Vantage Point')
        plt.subplot(2,2,4)
        plt.imshow(self.slidingVis>0) 
        plt.contour(np.maximum(psi,self.psiQueue[-1]),0)
        plt.plot(xP[-j:,1]/dx,xP[-j:,0]/dx,'b.')
        plt.title('Sliding Window Visibility')

        fig = plt.gcf()
        fig.set_size_inches(9, 9)
        fig.savefig(name, dpi=300, bbox_inches='tight')
        plt.close()

        #plt.imshow(self.totalVis)
        #plt.savefig('total_vis.png', dpi=300, bbox_inches='tight')

        #plt.show()
        #plt.pause(2); plt.clf()


    def compute_occ_time(self, xP, xE, knownEnv = True):
        [iP, jP, iE, jE] = self.position_to_sub(xP,xE)

    
        m = self.m
        dx = self.dx
        speed = self.speed
        phi = self.phi

        # TODO: this portion is for unknown env
        if not knownEnv:
            self.vP = self.vP * (self.totalVis>0)

        self.psiE, _ = computeVisibility(self.phi, -1e12, xE, dx)
 
        if self.psiE[iP,jP] < 0:
            exit()

        occTime = np.zeros((m,m))
        for i in range(m):
            for j in range(m):
                if self.psiE[i,j] <= 0:
                    continue
                x0 = np.array([[i,j]])*dx
                psi, _ = computeVisibility(phi, -1e12, x0, dx)
        
                shadow = 1.0 * (psi >= 0) + 1.0 * (phi <= 0)
                shadow = shadow - 0.5
                time = travel_time(shadow, self.vE * speed, dx = dx) 
                occTime[i,j] = time[iE,jE]


        wP = np.ones((m,m)) 
        wP[iP,jP] = 0
        uP = travel_time(wP, self.vP * speed, dx = dx)

        R0 = occTime - uP
        
        for i in range(20):
            uP = travel_time(wP, self.vP * speed*(R0.data>=0)*(1-R0.mask), dx = dx)
            R = occTime - uP
            if (np.abs(1*(R>=0)-1*(R0>=0))).sum() == 0:
                break
            else:
                R0 = 1*R
    
        #uP = travel_time(wP, self.vP * speed*(R>=0), dx = dx)
        uP = travel_time(wP, self.vP * speed*(1-R.mask), dx = dx)
        tStar = (occTime * (R>=0) ).max() 
        wE = np.ones((m,m)) 
        wE[iE,jE] = 0
        uE = travel_time(wE, self.vE * speed, dx = dx)
        #uE = uE*(uE<=tStar)        
    
        self.uP = uP
        self.uE = uE
        self.occTime = occTime    
        self.R = R
        self.tStar = tStar
        self.iP =  iP
        self.jP =  jP
        self.iE =  iE
        self.jE =  jE

         
if __name__ == '__main__':
    
    sP = 100
    sE = 150
    e = env('figures/unnamed.png', vP = 3) #vP is pursuer speed
    path = e.compute_reach_gain_path(sP, sE)
    print(path)

