#!/usr/bin/env python
import argparse
import random

import time
import math
import numpy
import ViconTrackerPoseHandler as vt
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import sys

def plot_init_():
	# initialize plot settings
	#mapBounds = [-3,3,-3,3]
	tickSpacing =1
	markerStyle = 'bo'
	# make plotinteractive to allow for dynamic plots
	plt.ion()
	# create plot objects
	ax = plt.gca()
	mp = plt.plot(numpy.zeros((1,numAll)),numpy.zeros((1,numAll)),markerStyle)
	# edit Axes
	plt.axis('equal')
	#plt.axis(mapBounds)
	ax.xaxis.set_major_locator(ticker.MultipleLocator(tickSpacing))
	ax.yaxis.set_major_locator(ticker.MultipleLocator(tickSpacing))

	wallRange = 3.0
	MAP = numpy.matrix([[-wallRange,-wallRange,wallRange,-wallRange],[wallRange,-wallRange,wallRange,wallRange+0.6],\
						[wallRange,wallRange+0.6,-wallRange,wallRange+0.6],[-wallRange,wallRange+0.6,-wallRange,-wallRange],\
						[-1.0,1.0,0,1.0],[-1.0,1.0,-1.0,-1.0],[-1.0,-1.0,0,-1.0],[2.0,1.0,3.0,1.0],\
						[2.0,-1.0,3.0,-1.0]])
	nWalls = MAP.shape[0]

	door = numpy.matrix([[0,1.0,0,-1.0],[2.0,1.0,2.0,-1.0]])
	nDoors = door.shape[0]

	#nWalls = 5

	plt.text(-2.2,2.8,'1',style='italic',fontsize = 20)
	plt.text(2.5,2.8,'2',style='italic',fontsize = 20)
	plt.text(-0.5,0,'3',style='italic',fontsize = 20)
	plt.text(2.5,0,'4',style='italic',fontsize = 20)
	plt.text(-2.0,-2.0,'5',style='italic',fontsize = 20)
	plt.text(2.0,-2.0,'6',style='italic',fontsize = 20)
	plt.text(-2.0,0,'7',style='italic',fontsize = 20)

	for i in range(nWalls):
		plt.plot([MAP[i,0],MAP[i,2]],[MAP[i,1],MAP[i,3]],'k-',lw=2)
	
	for i in range(nDoors):
		plt.plot([door[i,0],door[i,2]],[door[i,1],door[i,3]],'c--',lw=1)

	regionCenters = numpy.matrix([[-2.5,2.5,-0.5,2.5,-2.5,2.5,-2.0],[3.1,2.8,0,0,-2.5,-2.5,0]])
	tol = 0.1
	offsetVector = numpy.matrix([[0.9+tol,0.5+tol,0.3+tol,0.3+tol,0.8+tol,0.8+tol,0.4+tol],[0.9+tol,0.6+tol,0.8+tol,0.8+tol,0.8+tol,0.8+tol,0.8+tol]])
	
	nCenters = regionCenters.shape[1]
	for i in range(nCenters):
		plt.plot([regionCenters[0,i]+offsetVector[0,i],regionCenters[0,i]+offsetVector[0,i]],[regionCenters[1,i]-offsetVector[1,i],regionCenters[1,i]+offsetVector[1,i]],'g--',lw=0.6)
		plt.plot([regionCenters[0,i]+offsetVector[0,i],regionCenters[0,i]-offsetVector[0,i]],[regionCenters[1,i]+offsetVector[1,i],regionCenters[1,i]+offsetVector[1,i]],'g--',lw=0.6)
		plt.plot([regionCenters[0,i]-offsetVector[0,i],regionCenters[0,i]-offsetVector[0,i]],[regionCenters[1,i]-offsetVector[1,i],regionCenters[1,i]+offsetVector[1,i]],'g--',lw=0.6)
		plt.plot([regionCenters[0,i]-offsetVector[0,i],regionCenters[0,i]+offsetVector[0,i]],[regionCenters[1,i]-offsetVector[1,i],regionCenters[1,i]-offsetVector[1,i]],'g--',lw=0.6)
	    
	plt.show()

	return (ax,mp)
def updatePlot(mapPlot, viconArray, text,circles):
	xPos = []
	yPos = []
	for i in range(len(viconArray)):
		v = viconArray[i]
		pos = v.getPose()
		xPos.append(pos[0])
		yPos.append(pos[1])
		#print xPos
		#print yPos
		plt.setp(txt[i],'x',pos[0])
		plt.setp(txt[i],'y',pos[1])
		circle[i].center = pos[0],pos[1]


	plt.setp(mapPlot,'xdata',xPos)
	plt.setp(mapPlot,'ydata',yPos)
	plt.grid(True)
	plt.draw()

if __name__ == "__main__":
	viconStartNumHere = int(sys.argv[1])
	viconBegin = int(sys.argv[2])
	viconEnd = int(sys.argv[3])
	diff = viconBegin - 1
	numAll = viconEnd-viconBegin+1

	safetyRad = 0.3

	(ax,mp) = plot_init_()

	vicon=[[]]*numAll
	txt = [[]]*numAll
	circle = [[]]*numAll

	for i in range(numAll):
		vicon[i] = vt.ViconTrackerPoseHandler(None, None, "",viconStartNumHere+diff+i, 'Sphero'+str(i+1))
		txt[i] = plt.text(0,0,str(i+diff+1),fontsize = 12)
		circle[i] = plt.Circle((0,0),safetyRad/2,color ='b', fill=False)
		ax.add_artist(circle[i])
	
	while True:
		try:
			updatePlot(mp,vicon,txt,circle)
			time.sleep(0.05)
		except KeyboardInterrupt:
			break




