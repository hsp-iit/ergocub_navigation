#!/usr/bin/env python3
import os                                                                      # For obtaining file directories
import pandas                                                                  # For reading csv files
import matplotlib.pyplot 
import numpy
from matplotlib.patches import Ellipse
from matplotlib.patches import Circle
from matplotlib.patches import Patch
from matplotlib.patches import PathPatch
from matplotlib.path import Path as Path2

import math
from time import sleep
from matplotlib.animation import FuncAnimation
from pathlib import Path
matplotlib.pyplot.close('all')
import sys
import csv
import math
path = os.path.abspath(os.path.join(Path.home(),sys.argv[1]))
allDataRaw = pandas.read_csv(path, header=None) # Read Raw Data CSV Obtained After Running Bag Files
allDataRaw.columns = ("x1","y1","x2","y2","hx1","hy1","hx2","hy2","rx1","ry1","rhx1","rhy1","yaw","at1","at2","ht1","ht2","rt1","rht1")
# x1,y1 = Human Left Elbow Position in Odom Frame 
# x2,y2 = Human Right Elbow in Odom Frame
# at1,at2 = Time Stamp of Arm Data
# hx1, hy2, hx2, hxy2 = Human Extreme Data Preferably in Odom Frame
# rx,ry1, rhx1, rhy1 = Robot Odom to Base Transform and Robot Base to Head Transform Data
# rt1, rht1  = Robot transfrom time stampes.
startFrame = int(sys.argv[2]) # Start Frame to Begin Animation
endFrame  = int(sys.argv[3]) # Frame to End Animation
lowPassAlpha = float(sys.argv[4]) # Low Pass Filter For X-Position
lowPassAlphaY = float(sys.argv[5])# Low Pass Filter For X-Position
timeTolerance = float(sys.argv[6])# Allowed Time Tolerance Between Data to Be Matched

def find_nearest(array, value): 
    array = numpy.asarray(array)
    idx = (numpy.abs(array - value)).argmin()
    return idx

init_time = allDataRaw["rt1"].values[0]
for i in range(allDataRaw["rt1"].values.shape[0]):
    allDataRaw["at1"].values[i] = allDataRaw["at1"].values[i] - init_time
    allDataRaw["at2"].values[i] = allDataRaw["at2"].values[i] - init_time
    allDataRaw["ht1"].values[i] = allDataRaw["ht1"].values[i] - init_time
    allDataRaw["ht2"].values[i] = allDataRaw["ht2"].values[i] - init_time
    allDataRaw["rt1"].values[i] = allDataRaw["rt1"].values[i] - init_time
    allDataRaw["rht1"].values[i] = allDataRaw["rht1"].values[i] - init_time


alpha = lowPassAlpha
alphay = lowPassAlphaY # Pefroming Low Pass Filter on the Human Extreme Values.
for i in range(allDataRaw["hx1"].values.shape[0]-1):
    allDataRaw.at[i+1,'hx1'] =  alpha * allDataRaw.at[i+1,'hx1'] + (1-alpha)* allDataRaw.at[i,'hx1']
    allDataRaw.at[i+1,'hy1'] =  alphay * allDataRaw.at[i+1,'hy1'] + (1-alphay)* allDataRaw.at[i,'hy1']
    allDataRaw.at[i+1,'hx2'] =  alpha * allDataRaw.at[i+1,'hx2'] + (1-alpha)* allDataRaw.at[i,'hx2']
    allDataRaw.at[i+1,'hy2'] =  alphay * allDataRaw.at[i+1,'hy2'] + (1-alphay)* allDataRaw.at[i,'hy2']



    
dataList = list()
# Putting All the Porcessed Data after Matching Appropriate Transfroms
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
    robot_head_set = False
    
    hx1 = allDataRaw["hx1"].values[i]
    hy1 = allDataRaw["hy1"].values[i]
    
    
    if abs(allDataRaw["ht1"].values[i]-allDataRaw["rt1"].values[find_nearest(allDataRaw["rt1"].values,allDataRaw["ht1"].values[i])]) < timeTolerance:
        rx1 = allDataRaw["rx1"].values[find_nearest(allDataRaw["rt1"].values,allDataRaw["ht1"].values[i])]
        ry1 = allDataRaw["ry1"].values[find_nearest(allDataRaw["rt1"].values,allDataRaw["ht1"].values[i])]
        robot_set = True

    if abs(allDataRaw["ht1"].values[i]-allDataRaw["at1"].values[find_nearest(allDataRaw["at1"].values, allDataRaw["ht1"].values[i])]) < timeTolerance:
        x1 = allDataRaw["x1"].values[find_nearest(allDataRaw["at1"].values,allDataRaw["ht1"].values[i])]
        y1 = allDataRaw["y1"].values[find_nearest(allDataRaw["at1"].values,allDataRaw["ht1"].values[i])]
        left_arm_set = True
        

    if abs(allDataRaw["ht1"].values[i]-allDataRaw["at2"].values[find_nearest(allDataRaw["at2"].values,allDataRaw["ht1"].values[i])]) < timeTolerance:
        x2 = allDataRaw["x2"].values[find_nearest(allDataRaw["at2"].values,allDataRaw["ht1"].values[i])]
        y2 = allDataRaw["y2"].values[find_nearest(allDataRaw["at2"].values,allDataRaw["ht1"].values[i])]
        right_arm_set = True
    
    if abs(allDataRaw["ht1"].values[i]-allDataRaw["ht2"].values[find_nearest(allDataRaw["ht2"].values,allDataRaw["ht1"].values[i])]) < timeTolerance:
        hx2 = allDataRaw["hx2"].values[find_nearest(allDataRaw["ht2"].values,allDataRaw["ht1"].values[i])]
        hy2 = allDataRaw["hy2"].values[find_nearest(allDataRaw["ht2"].values,allDataRaw["ht1"].values[i])]
        right_human_set = True
    
    if abs(allDataRaw["ht1"].values[i]-allDataRaw["rht1"].values[find_nearest(allDataRaw["rht1"].values,allDataRaw["ht1"].values[i])]) < timeTolerance:
        rhx1 = allDataRaw["rhx1"].values[find_nearest(allDataRaw["rht1"].values,allDataRaw["ht1"].values[i])]
        rhy1 = allDataRaw["rhy1"].values[find_nearest(allDataRaw["rht1"].values,allDataRaw["ht1"].values[i])]
        robot_head_set = True

    

    if robot_set is True and left_arm_set is True and right_arm_set is True and right_human_set is True and robot_head_set is True:

        x1 = x1+rx1
        y1 = y1+ry1
        
        x2 = x2+rx1
        y2 = y2+ry1
   
        dataList.append({'x1': x1,'y1':y1, 'x2':x2,'y2':y2, 'hx1':hx1,'hy1':hy1, 'hx2':hx2,'hy2':hy2,'rx1':rx1, 'ry1':ry1})

allData=pandas.DataFrame(dataList) # Create New DataFrame and CSV with Processed Data
allData.columns = ("x1","y1","x2","y2","hx1","hy1","hx2","hy2","rx1","ry1")
allData.to_csv(os.path.abspath(os.path.join(Path.home(),"processed_"+sys.argv[1])), index=False, header=False)


robot_ellipses = None
human_ellipse = None
robot_centre = None

fig1, ax1 = matplotlib.pyplot.subplots(dpi=200)
matplotlib.pyplot.xlim([-1, 4])
matplotlib.pyplot.ylim([-1, 2])

# Make Initial Plot of Trajectories to Animate Them Later.
rbx_line, = ax1.plot(allData["rx1"].values,allData["ry1"].values, 'r--', label='ErgoCub Base Frame ')
relx_line, =  ax1.plot(allData["x1"].values,allData["y1"].values, linestyle='-.',color = '#fc9003', label='ErgoCub Elbows ')

rerx_line, =ax1.plot(allData["x2"].values,allData["y2"].values, linestyle='-.',color = '#fc9003')

hl_line, = ax1.plot(allData["hx1"].values,allData["hy1"].values, 'g-.', label = 'Human Extremes')
hr_line, = ax1.plot(allData["hx2"].values,allData["hy2"].values, 'g-.')
stopHuman = 506
def animate(i):
    
    global robot_ellipses
    global human_ellipse
    global hl_line
    global hr_line
    global rbx_line
    global relx_line
    global rerx_line

    
    if i>startFrame and i <endFrame:
        
        if robot_ellipses is not None:
            robot_ellipses.remove()
            
        if human_ellipse is not None and i<stopHuman:
            human_ellipse.remove()
       
        print(i)
        rlength = pow(pow(allData["x1"].values[i]-allData["x2"].values[i],2)+pow(allData["y1"].values[i]-allData["y2"].values[i],2),0.5)
        rangle = math.atan2(allData["y1"].values[i]-allData["y2"].values[i],allData["x1"].values[i]-allData["x2"].values[i])
        if rangle<0:
            rangle = rangle + 2*math.pi
        robot_ellipse=Ellipse((0.5*(allData["x1"].values[i]+allData["x2"].values[i]),0.5*(allData["y1"].values[i]+allData["y2"].values[i])),rlength, 0.45, numpy.degrees(rangle), edgecolor = '#0380fc', lw=2, facecolor='#0380fc', alpha=(i/(allData["rx1"].values.shape[0])),label='ErgoCub Occupancy')
        robot_ellipses = ax1.add_patch(robot_ellipse) # Animate Robot Ellipse

        if(i < stopHuman):
            hlength = pow(pow(allData["hx1"].values[i]-allData["hx2"].values[i],2)+pow(allData["hy1"].values[i]-allData["hy2"].values[i],2),0.5)
            hangle = math.atan2(allData["hy1"].values[i]-allData["hy2"].values[i],allData["hx1"].values[i]-allData["hx2"].values[i])
            if hangle<0:
                hangle = hangle + 2*math.pi
            hline = Ellipse((0.5*(allData["hx1"].values[i]+allData["hx2"].values[i]),0.5*(allData["hy1"].values[i]+allData["hy2"].values[i])),hlength,0.55,numpy.degrees(hangle), edgecolor = '#008000', lw=2, facecolor='#008000', label = 'Human Occupancy' )

            human_ellipse = ax1.add_patch(hline) # Animate Human Ellipse
            
            hl_line.set_data(allData["hx1"].values[:i],allData["hy1"].values[:i])
            hr_line.set_data(allData["hx2"].values[:i],allData["hy2"].values[:i])
        rbx_line.set_data(allData["rx1"].values[:i],allData["ry1"].values[:i])
        relx_line.set_data(allData["x1"].values[:i],allData["y1"].values[:i])
        rerx_line.set_data(allData["x2"].values[:i],allData["y2"].values[:i]) 

        matplotlib.pyplot.legend(loc="upper left")

        return [robot_ellipses, human_ellipse, hl_line, hr_line, rbx_line, relx_line, rerx_line]


anim = FuncAnimation(fig1, animate, frames = allData["x1"].values.shape[0], interval = 30, repeat=False,cache_frame_data=False) 
anim
matplotlib.pyplot.legend(loc="upper left")
matplotlib.pyplot.show()





