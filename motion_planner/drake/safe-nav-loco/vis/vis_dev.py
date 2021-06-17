#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import mpl_toolkits.mplot3d.axes3d as p3
from mpl_toolkits.mplot3d.art3d import Poly3DCollection,Line3DCollection


log_apex = np.loadtxt("log_apex.txt")
log_d = np.loadtxt("log_d.txt")
log_switch = np.loadtxt("log_switch.txt")
log_p_foot = np.loadtxt("log_p_foot.txt")
log_COM = np.loadtxt("log_COM.txt")
log_l_foot = np.loadtxt("log_l_foot.txt")
log_r_foot = np.loadtxt("log_r_foot.txt")
log_obstacle = np.loadtxt("log_obstacle.txt")
log_waypoint = np.loadtxt("log_waypoint.txt")

cellz = 2.70351;
stairz = 4*0.104;
fig = plt.figure()
'''
ax1 = fig.add_subplot(1, 1, 1, projection="3d")
ax1.set_aspect('equal')
ax1.set_xlabel('x')
ax1.set_ylabel('y')
ax1.set_zlabel('z')
ax1.set_xlim(-7*cellz, 7*cellz)
ax1.set_ylim(-13*cellz, 13*cellz)
ax1.set_zlim(-5, 5)

ax1.axis('equal')
ax1.set_aspect(1)
'''
ax1=fig.add_subplot(1,1,1)
ax1.axis('equal')

#ax2=fig.add_subplot(3,1,2)
#ax2.axis('equal')

#ax3=fig.add_subplot(3,1,3)


x_d = log_d[:, 0]
y_d = log_d[:, 1]
z_d = log_d[:, 2]
ax1.plot(x_d, y_d, label="High-level waypoints", color="black", marker="o")

#x_w = log_waypoint[:, 0]
#y_w = log_waypoint[:, 1]
#z_w = log_waypoint[:, 2]
#ax1.plot(x_w, y_w, color="orange", marker="o")

x_switch = log_switch[:, 0]
y_switch = log_switch[:, 1]
z_switch = log_switch[:, 2]
xd_switch = log_switch[:,3]
x_p_foot = log_p_foot[:, 0]
y_p_foot = log_p_foot[:, 1]
z_p_foot = log_p_foot[:, 2]

ax1.scatter(x_p_foot, y_p_foot, label="foot placement", color="purple", marker="o")
#ax2.scatter(x_p_foot, x_p_foot*0, label="foot placement", color="purple", marker="o")
#ax3.scatter(y_p_foot, y_p_foot*0, label="foot placement", color="purple", marker="o")

x_COM = log_COM[:, 0]
y_COM = log_COM[:, 1]
z_COM = log_COM[:, 2]
xd_COM = log_COM[:, 3]
yd_COM = log_COM[:, 4]
zd_COM = log_COM[:, 5]

ax1.plot(x_COM, y_COM, linewidth=2, label="walker CoM trajectory", color="red")
#ax1.plot(log_obstacle[:,0], log_obstacle[:,1], linewidth=2, label="walker CoM trajectory", color="green")
#ax2.plot(x_COM,xd_COM)
#ax3.plot(y_COM,yd_COM)


x_apex = log_apex[:, 0]
y_apex = log_apex[:, 1]
z_apex = log_apex[:, 2]
xd_apex = log_apex[:,4]

ax1.scatter(x_apex, y_apex, linewidth=2, label="Apex", color="red",marker="o")
#ax1.scatter(x_switch, y_switch, linewidth=2, label="Apex", color="black",marker="o")
#ax2.scatter(x_apex, xd_apex, label="Apex", color="red",marker="o")
#ax2.scatter(x_switch, xd_switch, label="Apex", color="red",marker="o")

x_l_foot = log_l_foot[:, 0]
y_l_foot = log_l_foot[:, 1]
z_l_foot = log_l_foot[:, 2]
ax1.plot(x_l_foot, y_l_foot, label="left foot", color="blue")

x_r_foot = log_r_foot[:, 0]
y_r_foot = log_r_foot[:, 1]
z_r_foot = log_r_foot[:, 2]
ax1.plot(x_r_foot, y_r_foot, label="right foot", color="yellow")

'''
x_ground, y_ground = np.meshgrid(np.linspace(0,5*cellz, 100), np.linspace(0,8*cellz, 100))
X_ground = x_ground.T
Y_ground = y_ground.T
Z_ground = 0* np.ones((100, 100))
ax1.plot_surface(X_ground, Y_ground, Z_ground, color="w", alpha=0.5)
'''
'''
x_ground2, y_ground2 = np.meshgrid(np.linspace(0,5*cellz, 100), np.linspace(8*cellz,11*cellz, 100))
X_ground2 = x_ground2.T
Y_ground2 = y_ground2.T
Z_ground2 = 0* np.ones((100, 100))
ax1.plot_surface(X_ground2, Y_ground2, Z_ground2, color="w", alpha=0.5)
'''
'''
verts1 = [(0, cellz, 0), (cellz, cellz, 0), (0, 2*cellz, 0), (cellz, 2*cellz, 0), (0, cellz, 0.5), (cellz, cellz, 0.5), (0, 2*cellz, 0.5), (cellz, 2*cellz, 0.5)]
faces1 = [[0, 1, 3, 2], [4, 5, 7, 6], [0, 1, 5, 4], [2, 3, 7, 6], [0, 2, 6, 4], [1, 3, 7, 5]]
poly3d1 = [[verts1[vert_id] for vert_id in face] for face in faces1]
ax1.add_collection3d(Poly3DCollection(poly3d1, facecolors='gray', linewidths=1, alpha=0.5))
ax1.add_collection3d(Line3DCollection(poly3d1, colors='k', linewidths=0.5, linestyles=':'))

verts1 = [(2*cellz, 3*cellz, 0), (3*cellz, 3*cellz, 0), (2*cellz, 4*cellz, 0), (3*cellz, 4*cellz, 0), (2*cellz, 3*cellz, 0.5), (3*cellz, 3*cellz, 0.5), (2*cellz, 4*cellz, 0.5), (3*cellz, 4*cellz, 0.5)]
faces1 = [[0, 1, 3, 2], [4, 5, 7, 6], [0, 1, 5, 4], [2, 3, 7, 6], [0, 2, 6, 4], [1, 3, 7, 5]]
poly3d1 = [[verts1[vert_id] for vert_id in face] for face in faces1]
ax1.add_collection3d(Poly3DCollection(poly3d1, facecolors='gray', linewidths=1, alpha=0.5))
ax1.add_collection3d(Line3DCollection(poly3d1, colors='k', linewidths=0.5, linestyles=':'))

verts1 = [(4*cellz, 3*cellz, 0), (5*cellz, 3*cellz, 0), (4*cellz, 4*cellz, 0), (5*cellz, 4*cellz, 0), (4*cellz, 3*cellz, 0.5), (5*cellz, 3*cellz, 0.5), (4*cellz, 4*cellz, 0.5), (5*cellz, 4*cellz, 0.5)]
faces1 = [[0, 1, 3, 2], [4, 5, 7, 6], [0, 1, 5, 4], [2, 3, 7, 6], [0, 2, 6, 4], [1, 3, 7, 5]]
poly3d1 = [[verts1[vert_id] for vert_id in face] for face in faces1]
ax1.add_collection3d(Poly3DCollection(poly3d1, facecolors='gray', linewidths=1, alpha=0.5))
ax1.add_collection3d(Line3DCollection(poly3d1, colors='k', linewidths=0.5, linestyles=':'))

verts1 = [(0, 7*cellz, 0), (2*cellz, 7*cellz, 0), (0, 8*cellz, 0), (2*cellz, 8*cellz, 0), (0, 7*cellz, 0.8), (2*cellz, 7*cellz, 0.8), (0, 8*cellz, 0.8), (2*cellz, 8*cellz, 0.8)]
faces1 = [[0, 1, 3, 2], [4, 5, 7, 6], [0, 1, 5, 4], [2, 3, 7, 6], [0, 2, 6, 4], [1, 3, 7, 5]]
poly3d1 = [[verts1[vert_id] for vert_id in face] for face in faces1]
ax1.add_collection3d(Poly3DCollection(poly3d1, facecolors='gray', linewidths=1, alpha=0.5))
ax1.add_collection3d(Line3DCollection(poly3d1, colors='k', linewidths=0.8, linestyles=':'))

verts1 = [(3*cellz, 7*cellz, 0), (5*cellz, 7*cellz, 0), (3*cellz, 8*cellz, 0), (5*cellz, 8*cellz, 0), (3*cellz, 7*cellz, 0.8), (5*cellz, 7*cellz, 0.8), (3*cellz, 8*cellz, 0.8), (5*cellz, 8*cellz, 0.8)]
faces1 = [[0, 1, 3, 2], [4, 5, 7, 6], [0, 1, 5, 4], [2, 3, 7, 6], [0, 2, 6, 4], [1, 3, 7, 5]]
poly3d1 = [[verts1[vert_id] for vert_id in face] for face in faces1]
ax1.add_collection3d(Poly3DCollection(poly3d1, facecolors='gray', linewidths=1, alpha=0.5))
ax1.add_collection3d(Line3DCollection(poly3d1, colors='k', linewidths=0.8, linestyles=':'))
'''
#Stairs
'''
verts1 = [(2*cellz, 7*cellz, 0), (3*cellz, 7*cellz, 0), (2*cellz, 8*cellz, 0), (3*cellz, 8*cellz, 0), (2*cellz, 7*cellz, 0.1), (3*cellz, 7*cellz, 0.1), (2*cellz, 8*cellz, 0.1), (3*cellz, 8*cellz, 0.1)]
faces1 = [[0, 1, 3, 2], [4, 5, 7, 6], [0, 1, 5, 4], [2, 3, 7, 6], [0, 2, 6, 4], [1, 3, 7, 5]]
poly3d1 = [[verts1[vert_id] for vert_id in face] for face in faces1]
ax1.add_collection3d(Poly3DCollection(poly3d1, facecolors='green', linewidths=1, alpha=1))
ax1.add_collection3d(Line3DCollection(poly3d1, colors='k', linewidths=0.5, linestyles=':'))

verts1 = [(2*cellz, 7*cellz+stairz, 0), (3*cellz, 7*cellz+stairz, 0), (2*cellz, 8*cellz, 0), (3*cellz, 8*cellz, 0), (2*cellz, 7*cellz+stairz, 0.2), (3*cellz, 7*cellz+stairz, 0.2), (2*cellz, 8*cellz, 0.2), (3*cellz, 8*cellz, 0.2)]
faces1 = [[0, 1, 3, 2], [4, 5, 7, 6], [0, 1, 5, 4], [2, 3, 7, 6], [0, 2, 6, 4], [1, 3, 7, 5]]
poly3d1 = [[verts1[vert_id] for vert_id in face] for face in faces1]
ax1.add_collection3d(Poly3DCollection(poly3d1, facecolors='green', linewidths=1, alpha=1))
ax1.add_collection3d(Line3DCollection(poly3d1, colors='k', linewidths=0.5, linestyles=':'))

verts1 = [(2*cellz, 7*cellz+stairz*2, 0), (3*cellz, 7*cellz+stairz*2, 0), (2*cellz, 8*cellz, 0), (3*cellz, 8*cellz, 0), (2*cellz, 7*cellz+stairz*2, 0.3), (3*cellz, 7*cellz+stairz*2, 0.3), (2*cellz, 8*cellz, 0.3), (3*cellz, 8*cellz, 0.3)]
faces1 = [[0, 1, 3, 2], [4, 5, 7, 6], [0, 1, 5, 4], [2, 3, 7, 6], [0, 2, 6, 4], [1, 3, 7, 5]]
poly3d1 = [[verts1[vert_id] for vert_id in face] for face in faces1]
ax1.add_collection3d(Poly3DCollection(poly3d1, facecolors='green', linewidths=1, alpha=1))
ax1.add_collection3d(Line3DCollection(poly3d1, colors='k', linewidths=0.5, linestyles=':'))

verts1 = [(2*cellz, 7*cellz+stairz*3, 0), (3*cellz, 7*cellz+stairz*3, 0), (2*cellz, 8*cellz, 0), (3*cellz, 8*cellz, 0), (2*cellz, 7*cellz+stairz*3, 0.4), (3*cellz, 7*cellz+stairz*3, 0.4), (2*cellz, 8*cellz, 0.4), (3*cellz, 8*cellz, 0.4)]
faces1 = [[0, 1, 3, 2], [4, 5, 7, 6], [0, 1, 5, 4], [2, 3, 7, 6], [0, 2, 6, 4], [1, 3, 7, 5]]
poly3d1 = [[verts1[vert_id] for vert_id in face] for face in faces1]
ax1.add_collection3d(Poly3DCollection(poly3d1, facecolors='green', linewidths=1, alpha=1))
ax1.add_collection3d(Line3DCollection(poly3d1, colors='k', linewidths=0.5, linestyles=':'))

verts1 = [(2*cellz, 7*cellz+stairz*4, 0), (3*cellz, 7*cellz+stairz*4, 0), (2*cellz, 8*cellz, 0), (3*cellz, 8*cellz, 0), (2*cellz, 7*cellz+stairz*4, 0.5), (3*cellz, 7*cellz+stairz*4, 0.5), (2*cellz, 8*cellz, 0.5), (3*cellz, 8*cellz, 0.5)]
faces1 = [[0, 1, 3, 2], [4, 5, 7, 6], [0, 1, 5, 4], [2, 3, 7, 6], [0, 2, 6, 4], [1, 3, 7, 5]]
poly3d1 = [[verts1[vert_id] for vert_id in face] for face in faces1]
ax1.add_collection3d(Poly3DCollection(poly3d1, facecolors='green', linewidths=1, alpha=1))
ax1.add_collection3d(Line3DCollection(poly3d1, colors='k', linewidths=0.5, linestyles=':'))

verts1 = [(2*cellz, 7*cellz+stairz*5, 0), (3*cellz, 7*cellz+stairz*5, 0), (2*cellz, 8*cellz, 0), (3*cellz, 8*cellz, 0), (2*cellz, 7*cellz+stairz*5, 0.6), (3*cellz, 7*cellz+stairz*5, 0.6), (2*cellz, 8*cellz, 0.6), (3*cellz, 8*cellz, 0.6)]
faces1 = [[0, 1, 3, 2], [4, 5, 7, 6], [0, 1, 5, 4], [2, 3, 7, 6], [0, 2, 6, 4], [1, 3, 7, 5]]
poly3d1 = [[verts1[vert_id] for vert_id in face] for face in faces1]
ax1.add_collection3d(Poly3DCollection(poly3d1, facecolors='green', linewidths=1, alpha=1))
ax1.add_collection3d(Line3DCollection(poly3d1, colors='k', linewidths=0.5, linestyles=':'))
'''
#ax1.view_init(azim=0, elev=90)
plt.axis('on')

plt.show()
