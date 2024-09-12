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
allDataRaw = pandas.read_csv(path, header=None)
allDataRaw.columns = ("x1","y1","x2","y2","hx1","hy1","hx2","hy2","rx1","ry1","at1","at2","ht1","ht2","rt1")

def find_nearest(array, value):
    array = numpy.asarray(array)
    idx = (numpy.abs(array - value)).argmin()
    return idx

for i in range(allDataRaw["rt1"].values.shape[0]):
    allDataRaw["at1"].values[i] = -allDataRaw["rt1"].values[0] + allDataRaw["at1"].values[i]
    allDataRaw["at2"].values[i] = -allDataRaw["rt1"].values[0] + allDataRaw["at2"].values[i]
    allDataRaw["ht1"].values[i] = -allDataRaw["rt1"].values[0] + allDataRaw["ht1"].values[i]
    allDataRaw["ht2"].values[i] = -allDataRaw["rt1"].values[0] + allDataRaw["ht2"].values[i]
    allDataRaw["rt1"].values[i] = -allDataRaw["rt1"].values[0] + allDataRaw["rt1"].values[i]

dataList = list()
for i in range(allDataRaw["ht1"].values.shape[0]):
    x1=None
    y1=None
    hx1=None
    hy1=None

    x2=None
    y2=None
    hx2=None
    hy2=None

    rx1=None
    ry1=None
    
    robot_set = False
    left_arm_set = False
    right_arm_set = False
    right_human_set = False
    left_human_set = False
    
    hx1 = allDataRaw["hx1"].values[i]
    hy1 = allDataRaw["hy1"].values[i]
    
    if abs(allDataRaw["ht1"].values[i]-allDataRaw["rt1"].values[find_nearest(allDataRaw["rt1"].values,allDataRaw["ht1"].values[i])]) < 0.01:
        rx1 = allDataRaw["rx1"].values[find_nearest(allDataRaw["rt1"].values,allDataRaw["ht1"].values[i])]
        ry1 = allDataRaw["ry1"].values[find_nearest(allDataRaw["rt1"].values,allDataRaw["ht1"].values[i])]
        robot_set = True

    if abs(allDataRaw["ht1"].values[i]-allDataRaw["at1"].values[find_nearest(allDataRaw["at1"].values, allDataRaw["ht1"].values[i])]) < 0.01:
        x1 = allDataRaw["x1"].values[find_nearest(allDataRaw["at1"].values,allDataRaw["ht1"].values[i])]
        y1 = allDataRaw["y1"].values[find_nearest(allDataRaw["at1"].values,allDataRaw["ht1"].values[i])]
        left_arm_set = True
        

    if abs(allDataRaw["ht1"].values[i]-allDataRaw["at2"].values[find_nearest(allDataRaw["at2"].values,allDataRaw["ht1"].values[i])]) < 0.01:
        x2 = allDataRaw["x2"].values[find_nearest(allDataRaw["at2"].values,allDataRaw["ht1"].values[i])]
        y2 = allDataRaw["y2"].values[find_nearest(allDataRaw["at2"].values,allDataRaw["ht1"].values[i])]
        right_arm_set = True
    
    if abs(allDataRaw["ht1"].values[i]-allDataRaw["ht2"].values[find_nearest(allDataRaw["ht2"].values,allDataRaw["ht1"].values[i])]) < 0.01:
        hx2 = allDataRaw["hx2"].values[find_nearest(allDataRaw["ht2"].values,allDataRaw["ht1"].values[i])]
        hy2 = allDataRaw["hy2"].values[find_nearest(allDataRaw["ht2"].values,allDataRaw["ht1"].values[i])]
        right_human_set = True

    

    if robot_set is True and left_arm_set is True and right_arm_set is True and right_human_set is True:
        hx1 = hx1+rx1
        hy1 = hy1+ry1
        
        hx2 = hx2+rx1
        hy2 = hy2+ry1
        
        x1 = x1+rx1
        y1 = y1+ry1
        
        x2 = x2+rx1
        y2 = y2+ry1
        
        dataList.append({'x1': x1,'y1':y1, 'x2':x2,'y2':y2, 'hx1':hx1,'hy1':hy1, 'hx2':hx2,'hy2':hy2,'rx1':rx1, 'ry1':ry1})

allData=pandas.DataFrame(dataList)
allData.columns = ("x1","y1","x2","y2","hx1","hy1","hx2","hy2","rx1","ry1")
allData.to_csv("/home/ecub_docker/processed_data.csv", sep='\t', encoding='utf-8', index=False, header=True)
kernel_size = 100
kernel = numpy.ones(kernel_size) / kernel_size    
hx1_values =  numpy.convolve(allData["hx1"].values, kernel, mode="same")
hy1_values =  numpy.convolve(allData["hy1"].values, kernel, mode="same")

hx2_values =  numpy.convolve(allData["hx2"].values, kernel, mode="same")
hy2_values =  numpy.convolve(allData["hy2"].values, kernel, mode="same")  
 




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
    global human_left
    global human_right
    
    if robot_ellipse is not None:
        robot_ellipse.remove()
    
    if human_ellipse is not None:
        human_ellipse.remove()
    
    if i < 100 or i>350:
        human_left = None
        human_right = None
    rlength = pow(pow(allData["x1"].values[i]-allData["x2"].values[i],2)+pow(allData["y1"].values[i]-allData["y2"].values[i],2),0.5)
    rangle = math.atan2(allData["y1"].values[i]-allData["y2"].values[i],allData["x1"].values[i]-allData["x2"].values[i])
    if rangle<0:
        rangle = rangle + 2*math.pi
    rline = Ellipse((0.5*(allData["x1"].values[i]+allData["x2"].values[i]),0.5*(allData["y1"].values[i]+allData["y2"].values[i])),rlength, 0.3, numpy.degrees(rangle), edgecolor = '#7348f1', lw=2, facecolor='none') 
    robot_ellipse = ax1.add_patch(rline)

    
    hlength = pow(pow(hx1_values[i]-hx2_values[i],2)+pow(hy1_values[i]-hy2_values[i],2),0.5)
    hangle = math.atan2(hy1_values[i]-hy2_values[i],hx1_values[i]-hx2_values[i])
    if hangle<0:
        hangle = hangle + 2*math.pi

    
    
    bline = Circle((allData["rx1"].values[i],allData["ry1"].values[i]), fill=True,facecolor='#FF0000', edgecolor='#FF0000', radius=0.01)
    robot_centre = ax1.add_patch(bline)
    
    if i>100 and i <350:
        hline = Ellipse((0.5*(hx1_values[i]+hx2_values[i]),0.5*(hy1_values[i]+hy2_values[i])),hlength,0.3,numpy.degrees(hangle), edgecolor = '#008000', lw=2, facecolor='none')
        human_ellipse = ax1.add_patch(hline)
        hleftline = Circle((hx1_values[i],hy1_values[i]),fill= False, facecolor='#0000FF', edgecolor='#0000FF', radius=0.005 )
        human_left = ax1.add_patch(hleftline)
        hrightline = Circle((hx2_values[i],hy2_values[i]),fill= False, facecolor='#0000FF', edgecolor='#0000FF', radius=0.005 )
        human_right = ax1.add_patch(hrightline)
    
    return [robot_ellipse,human_ellipse, robot_centre, human_left,human_right]


anim = FuncAnimation(fig1, animate, frames = allData["x1"].values.shape[0], interval = 30, repeat=False,cache_frame_data=False) 

#anim.save(os.path.abspath(os.path.join(Path.home(),sys.argv[2])), writer = 'ffmpeg', fps = 10) 
anim
matplotlib.pyplot.show()





