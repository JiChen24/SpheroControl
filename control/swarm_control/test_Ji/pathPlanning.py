# path planning for swarm

import numpy as np

import math


def createMap():
	wallRange = 3.0
	map = np.matrix([[-wallRange,-wallRange+0.35,wallRange,-wallRange+0.35],[wallRange,-wallRange,wallRange,wallRange+0.6],\
						[wallRange,wallRange+0.6,-wallRange,wallRange+0.6],[-wallRange,wallRange+0.6,-wallRange,-wallRange],\
						[-3.0,2.2,-1.4,2.2],[-3.0,-1.6,-1.4,-1.6],[-1.0,3.6,-1.0,2.0],[1.8,3.6,1.8,2.0],\
						[-0.8,-3.0,-0.8,-1.4],[0.2,-3.0,0.2,-1.4],[1.2,-3.0,1.2,-1.4],\
						[1.4,-3.0,1.4,-1.4],[2.4,-3.0,2.4,-1.4],[0.4,3.6,0.4,2.0]])
	return map

	
def createNodes():
	nodes = np.matrix([[0,1.8,-0.3,1.1,-0.3,0.7,2.1,-2.2,0.2],[0.2,0.2,2.6,2.6,-2.2,-2.2,-2.2,0.2,1.0]])
	#nodes2 = np.matrix([[-1.2,-1.2,-0.8,0.0,1.2,-0.6,0.6],[-1.8,1.8,-1.8,-1.8,-1.8,1.8,1.8]])
	#nodes = np.hstack([nodes,nodes2])
	return nodes

	
def insectPoint(x1,y1,x2,y2,x3,y3,x4,y4):
	denom = (y4-y3)*(x2-x1)-(x4-x3)*(y2-y1)
	if abs(denom) < 1e-6:
		isect = False
		ua = np.NaN
		x = np.NaN
		y = np.NaN
	else:
		ua = ((x4-x3)*(y1-y3)-(y4-y3)*(x1-x3))/denom
		ub = ((x2-x1)*(y1-y3)-(y2-y1)*(x1-x3))/denom
	
		if ua>=0 and ub>=0 and ua<=1 and ub<=1:
			isect = True
			x = x1+ua*(x2-x1)
			y = y1+ua*(y2-y1)
		else:
			isect = False
			x = np.NaN
			y = np.NaN
			
	return (isect,x,y,ua)
		

def lineQfree(q1,q2,map):
	# q1 q2 are 2-by-1 matrices or arrays
	nObs = len(map[:,1])
	
	isect = False
	for i in range(nObs):
		vertices = map[i,0]
		x1 = q1[0,0]
		y1 = q1[1,0]
		x2 = q2[0,0]
		y2 = q2[1,0]
		x3 = map[i,0]
		y3 = map[i,1]
		x4 = map[i,2]
		y4 = map[i,3]
		(isect,Xsect,Ysect,ua) = insectPoint(x1,y1,x2,y2,x3,y3,x4,y4)
		if isect == True:
			break
		
	return isect
		
	
def findPath(map,nodes,currentPose,goalPose):
	# currentPose and goalPose are 2-by-1 matrices or arrays
	nNodes = int(nodes.shape[1])
	
	# assuming always can find one node that connects the currentPose and goalPose
	isect = lineQfree(currentPose,goalPose,map)
    #print "isect=",isect

	if isect == True:
		waypoint = []
		for i in range(nNodes):
			pointTmp = nodes[:,i]

			print "pointTmp=",pointTmp

			isect1 = lineQfree(pointTmp,currentPose,map)
			isect2 = lineQfree(pointTmp,goalPose,map)

			#print "isect1=",isect1
			#print "isect2=",isect2
			'''if i==4:
			    if isect1==True or isect2==True:
				print "pointTmp=",pointTmp
				print "currentPose=",currentPose
				print "goalPose=",goalPose'''			
				
			if isect1 == False and isect2 == False:
				waypoint = pointTmp
				#print "waypoint=",waypoint
				#print "pointTmp=",pointTmp
				break
		if len(waypoint) == 0:
			waypoint = np.matrix([[0.0],[0.0]])
	else:
		waypoint = goalPose
	#print "waypoint=",waypoint
	return waypoint	


