#!/usr/bin/env python3
import os                                                                      # For obtaining file directories
import pandas                                                                  # For reading csv files
import matplotlib.pyplot 
import numpy
from matplotlib.patches import Ellipse
from matplotlib.patches import Circle
from matplotlib.patches import Patch


import math
from time import sleep
from matplotlib.animation import FuncAnimation
from pathlib import Path
matplotlib.pyplot.close('all')
import sys
import csv
import math
path = os.path.abspath(os.path.join(Path.home(),sys.argv[1]))
allData = pandas.read_csv(path, header=None)
allData.columns = ("x1","y1","x2","y2","hx1","hy1","hx2","hy2","rx1","ry1")

robot_ellipse = None
human_ellipse = None
robot_centre = None
human_left = None
human_right = None
fig1, ax1 = matplotlib.pyplot.subplots(dpi=200)
matplotlib.pyplot.xlim([-1, 5])
matplotlib.pyplot.ylim([-1, 2])


def animate(i):
    
    global robot_ellipse
    global human_ellipse
    global robot_centre
    
    if robot_ellipse is not None:
        robot_ellipse.remove()
    
    if human_ellipse is not None:
        human_ellipse.remove()
    
    rlength = pow(pow(allData["x1"].values[i]-allData["x2"].values[i],2)+pow(allData["y1"].values[i]-allData["y2"].values[i],2),0.5)
    rangle = math.atan2(allData["y1"].values[i]-allData["y2"].values[i],allData["x1"].values[i]-allData["x2"].values[i])
    if rangle<0:
        rangle = rangle + 2*math.pi
    rline = Ellipse((0.5*(allData["x1"].values[i]+allData["x2"].values[i]),0.5*(allData["y1"].values[i]+allData["y2"].values[i])),rlength, 0.3,numpy.degrees(rangle), edgecolor = '#7348f1', lw=2, facecolor='none') 
    robot_ellipse = ax1.add_patch(rline)

    
    hlength = pow(pow(allData["hx1"].values[i]-allData["hx2"].values[i],2)+pow(allData["hy1"].values[i]-allData["hy2"].values[i],2),0.5)
    hangle = math.atan2(allData["hy1"].values[i]-allData["hy2"].values[i],allData["hx1"].values[i]-allData["hx2"].values[i])
    if hangle<0:
        hangle = hangle + 2*math.pi

    hline = Ellipse((0.5*(allData["hx1"].values[i]+allData["hx2"].values[i]),0.5*(allData["hy1"].values[i]+allData["hy2"].values[i])),hlength,0.3,numpy.degrees(hangle), edgecolor = '#008000', lw=2, facecolor='none')
    human_ellipse = ax1.add_patch(hline)
    
    bline = Circle((allData["rx1"].values[i],allData["ry1"].values[i]), fill=True,facecolor='#FF0000', edgecolor='#FF0000', radius=0.01)
    robot_centre = ax1.add_patch(bline)
    
    hleftline = Circle((allData["hx1"].values[i],allData["hy1"].values[i]),fill= False, facecolor='#0000FF', edgecolor='#0000FF', radius=0.005 )
    human_left = ax1.add_patch(hleftline)
    hrightline = Circle((allData["hx2"].values[i],allData["hy2"].values[i]),fill= False, facecolor='#0000FF', edgecolor='#0000FF', radius=0.005 )
    human_right = ax1.add_patch(hrightline)
    
    return [robot_ellipse,human_ellipse, robot_centre, human_left,human_right]


anim = FuncAnimation(fig1, animate, frames = allData["x1"].values.shape[0], interval = 30, repeat=False,cache_frame_data=False) 

#anim.save(os.path.abspath(os.path.join(Path.home(),sys.argv[2])), writer = 'ffmpeg', fps = 10) 
anim
matplotlib.pyplot.show()




