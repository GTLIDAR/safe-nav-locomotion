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

fig = plt.figure()

ax1=fig.add_subplot(3,1,1)
ax1.axis('equal')

ax2=fig.add_subplot(3,1,2)
ax2.axis('equal')

ax3=fig.add_subplot(3,1,3)


x_d = log_d[:, 0]
y_d = log_d[:, 1]
z_d = log_d[:, 2]
ax1.scatter(x_d, y_d, label="High-level waypoints", color="yellow", marker="o")

x_w = log_waypoint[:, 0]
y_w = log_waypoint[:, 1]
z_w = log_waypoint[:, 2]
ax1.plot(x_w, y_w, color="orange", marker="o")

x_switch = log_switch[:, 0]
y_switch = log_switch[:, 1]
z_switch = log_switch[:, 2]

x_p_foot = log_p_foot[:, 0]
y_p_foot = log_p_foot[:, 1]
z_p_foot = log_p_foot[:, 2]

ax1.scatter(x_p_foot, y_p_foot, label="foot placement", color="purple", marker="o")
ax1.scatter(x_switch, y_switch, linewidth=2, label="Apex", color="black",marker="o")
ax2.scatter(x_p_foot, x_p_foot*0, label="foot placement", color="purple", marker="o")
ax3.scatter(y_p_foot, y_p_foot*0, label="foot placement", color="purple", marker="o")

x_COM = log_COM[:, 0]
y_COM = log_COM[:, 1]
z_COM = log_COM[:, 2]
xd_COM = log_COM[:, 3]
yd_COM = log_COM[:, 4]
zd_COM = log_COM[:, 5]

ax1.plot(x_COM, y_COM, linewidth=2, label="walker CoM trajectory", color="red")
ax2.plot(x_COM,xd_COM)
ax3.plot(y_COM,yd_COM)


x_apex = log_apex[:, 0]
y_apex = log_apex[:, 1]
z_apex = log_apex[:, 2]
xd_apex = log_apex[:,4]

ax1.scatter(x_apex, y_apex, linewidth=2, label="Apex", color="red",marker="o")
ax2.scatter(x_apex, xd_apex, label="Apex", color="red",marker="o")


x_l_foot = log_l_foot[:, 0]
y_l_foot = log_l_foot[:, 1]
z_l_foot = log_l_foot[:, 2]
#ax1.plot(x_l_foot, y_l_foot, z_l_foot*0+0.01, label="left foot", color="blue")

x_r_foot = log_r_foot[:, 0]
y_r_foot = log_r_foot[:, 1]
z_r_foot = log_r_foot[:, 2]
#ax1.plot(x_r_foot, y_r_foot, z_r_foot*0+0.01, label="right foot", color="yellow")


plt.axis('on')

plt.show()
