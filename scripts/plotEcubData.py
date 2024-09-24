#!/usr/bin/env python3
"""
File name: plotEcubData.py
Author: Vignesh Sushrutha Raghavan
Created: September 2024
Version: 1.0
Description: Script to further process collected human pose, robot data and animate the experiments.
"""
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
from scipy.interpolate import interp1d
from functools import reduce
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
#Get earliest time and bring all timestamps to 0 and onwards.
init_time = min(allDataRaw["rt1"].values[0],allDataRaw["rht1"].values[0],allDataRaw["at1"].values[0],allDataRaw["at2"].values[0],allDataRaw["ht1"].values[0],allDataRaw["ht2"].values[0])
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

# Create common timestamp and interpolate
common_time = reduce(numpy.union1d, [allDataRaw["at1"].values,allDataRaw["at2"].values,allDataRaw["ht1"].values,allDataRaw["ht2"].values, allDataRaw["rt1"].values, allDataRaw["rht1"].values])
print(common_time)
_,at1_unique = numpy.unique(allDataRaw["at1"].values, return_index=True)
_,at2_unique = numpy.unique(allDataRaw["at2"].values, return_index=True)
_,ht1_unique = numpy.unique(allDataRaw["ht1"].values, return_index=True)
_,ht2_unique = numpy.unique(allDataRaw["ht2"].values, return_index=True)
_,rt1_unique = numpy.unique(allDataRaw["rt1"].values, return_index=True)
_,rht1_unique = numpy.unique(allDataRaw["rht1"].values, return_index=True)


interp_x1 = interp1d(allDataRaw["at1"].values[at1_unique], allDataRaw["x1"].values[at1_unique], kind='cubic', fill_value="extrapolate")
interp_y1 = interp1d(allDataRaw["at1"].values[at1_unique], allDataRaw["y1"].values[at1_unique], kind='cubic', fill_value="extrapolate")

interp_x2 = interp1d(allDataRaw["at2"].values[at2_unique], allDataRaw["x2"].values[at2_unique], kind='cubic', fill_value="extrapolate")
interp_y2 = interp1d(allDataRaw["at2"].values[at2_unique], allDataRaw["y2"].values[at2_unique], kind='cubic', fill_value="extrapolate")


interp_hx1 = interp1d(allDataRaw["ht1"].values[ht1_unique], allDataRaw["hx1"].values[ht1_unique], kind='cubic', fill_value="extrapolate")
interp_hy1 = interp1d(allDataRaw["ht1"].values[ht1_unique], allDataRaw["hy1"].values[ht1_unique], kind='cubic', fill_value="extrapolate")

interp_hx2 = interp1d(allDataRaw["ht2"].values[ht2_unique], allDataRaw["hx2"].values[ht2_unique], kind='cubic', fill_value="extrapolate")
interp_hy2 = interp1d(allDataRaw["ht2"].values[ht2_unique], allDataRaw["hy2"].values[ht2_unique], kind='cubic', fill_value="extrapolate")

interp_rx1 = interp1d(allDataRaw["rt1"].values[rt1_unique], allDataRaw["rx1"].values[rt1_unique], kind='cubic', fill_value="extrapolate")
interp_ry1 = interp1d(allDataRaw["rt1"].values[rt1_unique], allDataRaw["ry1"].values[rt1_unique], kind='cubic', fill_value="extrapolate")

interp_rhx1 = interp1d(allDataRaw["rht1"].values[rht1_unique], allDataRaw["rhx1"].values[rht1_unique], kind='cubic', fill_value="extrapolate")
interp_rhy1 = interp1d(allDataRaw["rht1"].values[rht1_unique], allDataRaw["rhy1"].values[rht1_unique], kind='cubic', fill_value="extrapolate")

x1_interp = interp_x1(common_time)
y1_interp = interp_y1(common_time)
x2_interp = interp_x2(common_time)
y2_interp = interp_y2(common_time)
hx1_interp = interp_hx1(common_time)
hy1_interp = interp_hy1(common_time)
hx2_interp = interp_hx2(common_time)
hy2_interp = interp_hy2(common_time)
rx1_interp = interp_rx1(common_time)
ry1_interp = interp_ry1(common_time)
rhx1_interp = interp_rhx1(common_time)
rhy1_interp = interp_rhy1(common_time)

x1_interp+=rx1_interp
x2_interp+=rx1_interp
y1_interp+=ry1_interp
y2_interp+=ry1_interp
# Put interpolated and processed data into a dict to push to a new csv.
dataDict = {'x1':x1_interp,
            'y1':y1_interp,
            'x2':x2_interp,
            'y2':y2_interp,
            'hx1':hx1_interp,
            'hy1':hy1_interp,
            'hx2':hx2_interp,
            'hy2':hy2_interp,
            'rx1':rx1_interp,
            'ry1':ry1_interp,
            'time': common_time
}


allData=pandas.DataFrame(dataDict) # Create New DataFrame and CSV with Processed Data
allData.columns = ("x1","y1","x2","y2","hx1","hy1","hx2","hy2","rx1","ry1","time")
allData.to_csv(os.path.abspath(os.path.join(Path.home(),"processed_"+sys.argv[1])), index=False, header=False)


robot_ellipses = None
human_ellipse = None
robot_centre = None

fig1, ax1 = matplotlib.pyplot.subplots(dpi=200)
matplotlib.pyplot.xlim([-1, 4])
matplotlib.pyplot.ylim([-1.5, 2])
stopHuman = int(sys.argv[7])

# Make Initial Plot of Trajectories to Animate Them Later.
rbx_line, = ax1.plot(allData["rx1"].values[startFrame:endFrame],allData["ry1"].values[startFrame:endFrame], 'r--', label='ErgoCub Base Frame ')
relx_line, =  ax1.plot(allData["x1"].values[startFrame:endFrame],allData["y1"].values[startFrame:endFrame], linestyle='-.',color = '#fc9003', label='ErgoCub Elbows ')

rerx_line, =ax1.plot(allData["x2"].values[startFrame:endFrame],allData["y2"].values[startFrame:endFrame], linestyle='-.',color = '#fc9003')

hl_line, = ax1.plot(allData["hx1"].values[startFrame:stopHuman],allData["hy1"].values[startFrame:stopHuman], 'g-.', label = 'Human Extremes')
hr_line, = ax1.plot(allData["hx2"].values[startFrame:stopHuman],allData["hy2"].values[startFrame:stopHuman], 'g-.')
#matplotlib.pyplot.cla()
# The animate function to animate all the human extreme and robot elbow data.
def animate(i):
    
    global robot_ellipses
    global human_ellipse
    global hl_line
    global hr_line
    global rbx_line
    global relx_line
    global rerx_line

    # Start animating after cetain frames. Can be made better with Funcanimation args later
    if i>startFrame and i <endFrame: 
        
        if robot_ellipses is not None:
            robot_ellipses.remove()
            
        if human_ellipse is not None:
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
            hline = Ellipse((0.5*(allData["hx1"].values[i]+allData["hx2"].values[i]),0.5*(allData["hy1"].values[i]+allData["hy2"].values[i])),hlength,0.25,numpy.degrees(hangle), edgecolor = '#008000', lw=2, facecolor='#008000', label = 'Human Occupancy' )

            human_ellipse = ax1.add_patch(hline) # Animate Human Ellipse
            
            hl_line.set_data(allData["hx1"].values[startFrame:i],allData["hy1"].values[startFrame:i])
            hr_line.set_data(allData["hx2"].values[startFrame:i],allData["hy2"].values[startFrame:i])
        
        else:
            human_ellipse = None # Stop animating human ellipse as it flies off after human leaves FOV.
        rbx_line.set_data(allData["rx1"].values[startFrame:i],allData["ry1"].values[startFrame:i]) # Animate robot base line.
        relx_line.set_data(allData["x1"].values[startFrame:i],allData["y1"].values[startFrame:i]) # Animate robot left elbow line.
        rerx_line.set_data(allData["x2"].values[startFrame:i],allData["y2"].values[startFrame:i]) # Animate robot right line.

        matplotlib.pyplot.legend(loc="upper left")

        return [robot_ellipses, human_ellipse, hl_line, hr_line, rbx_line, relx_line, rerx_line]


anim = FuncAnimation(fig1, animate, frames = allData["x1"].values.shape[0], interval = 0.5, repeat=False,cache_frame_data=False) 
anim
matplotlib.pyplot.legend(loc="upper left")
matplotlib.pyplot.show()





