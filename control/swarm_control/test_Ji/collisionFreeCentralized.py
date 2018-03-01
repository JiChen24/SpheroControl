import numpy as np
#import scipy as sp

import math

import cvxopt
from cvxopt import solvers


	
def ZCBFfun(statei,statej,safeDis):

	
	sTmp1 = statei[0,0]-statej[0,0]
	sTmp2 = statei[0,1]-statej[0,1]
	
	deltaPij = np.matrix([[sTmp1],[sTmp2]])
	
	h = np.linalg.norm(deltaPij)-safeDis
	
	return h
	

def singleNominalController(currentState,goalPosition,kp,uLim):

	currentState = np.matrix(currentState)
	goalPosition = np.matrix(goalPosition)

	if len(currentState) < 2:
		currentState = currentState.transpose()
	
	if len(goalPosition) < 2:
		goalPosition = goalPosition.transpose()
	
	px = currentState[0,0]
	py = currentState[1,0]

	#print "goalPosition=",goalPosition
	
	pxGoal = goalPosition[0,0]
	pyGoal = goalPosition[1,0]
	
	vx = kp*(pxGoal-px)
	vy = kp*(pyGoal-py)
	
	if math.fabs(vx) > uLim or math.fabs(vy) > uLim:
		uLarge = max(math.fabs(vx),math.fabs(vy))
		vx = uLim*vx/uLarge
		vy = uLim*vy/uLarge
		
	return (vx,vy)
	
	
def closeStaticPoint(singleState,wall):


	x0 = singleState[0,0]
	y0 = singleState[1,0]
	
	x1 = wall[0,0]
	y1 = wall[0,1]
	x2 = wall[0,2]
	y2 = wall[0,3]
	
	t = ((x2-x1)*(x0-x1)+(y2-y1)*(y0-y1))/((x2-x1)**2+(y2-y1)**2)
	
	if t < 0:
		xClose = x1
		yClose = y1
	elif t <= 1:
		xClose = x1+t*(x2-x1)
		yClose = y1+t*(y2-y1)
	else:
		xClose = x2
		yClose = y2
		
	point = np.matrix([[xClose],[yClose]])
	
	return point



def quadProgramConstraints(states,sphero,map):
	lb = -sphero.inputLim[0]*np.ones((2*sphero.n,1))
	ub = -lb

	N1 = sphero.n*(sphero.n-1)/2
	A1 = np.zeros((N1,2*sphero.n))
	b1 = np.zeros((N1,1))

	count = -1
	for i in xrange(sphero.n-1):
		for j in xrange(i+1,sphero.n):
			count = count+1
			#
			A1[count,2*i] = -(states[0,i]-states[0,j])
			A1[count,2*i+1] = -(states[1,i]-states[1,j])
			A1[count,2*j] = states[0,i]-states[0,j]
			A1[count,2*j+1] = states[1,i]-states[1,j]
			#
			statei = states[:,i].transpose()
			statej = states[:,j].transpose()
			hij = ZCBFfun(statei,statej,sphero.Ds)
			b1[count,0] = sphero.gamma*(hij**3)*np.linalg.norm(statei-statej)
		
	
	nWall = len(map[:,0])
	
	N2 = sphero.n*nWall
	A2 = np.zeros((N2,2*sphero.n))
	b2 = np.zeros((N2,1))
	
	count = -1
	for i in xrange(sphero.n):
		for j in xrange(nWall):
			count = count+1

			closePoint = closeStaticPoint(states[:,i],map[j,:])

			A2[count,2*i] = -(states[0,i]-closePoint[0,0])
			A2[count,2*i+1] = -(states[1,i]-closePoint[1,0])

			statei = states[:,i].transpose()
			hij = ZCBFfun(statei,closePoint.transpose(),sphero.DsWall)

			b2[count,0] = sphero.gamma*(hij**3)*np.linalg.norm(statei-closePoint)
		
	
	A3 = np.identity(2*sphero.n)
	b3 = ub
	A4 = -np.identity(2*sphero.n)
	b4 = -lb
	
	A = np.vstack((A1,A2,A3,A4))	
	b = np.vstack((b1,b2,b3,b4))
	
	'''for k in xrange(len(A[:,0])):
		ATmp = np.sum(abs(A[k,:]))
		A[k,0] = A[k,0]/ATmp
		A[k,1] = A[k,1]/ATmp
		
		b[k,0] = b[k,0]/ATmp'''
		
	
	return (A,b)
	

def actualController(states,sphero,map):

	uNominal = np.zeros((2*sphero.n,1))
	uNominal = np.matrix(uNominal)

	#print "uNominal=",uNominal

	for i in xrange(sphero.n):
		(uxTmp,uyTmp) = singleNominalController(states[:,i],sphero.goalPose[:,i],sphero.kp,sphero.inputLim[i])
		uNominal[2*i,0] = uxTmp
		uNominal[2*i+1,0] = uyTmp

	
	H = np.identity(2*sphero.n)
	f = -uNominal
	
	(A,b) = quadProgramConstraints(states,sphero,map)
	
	solvers.options['show_progress'] = False
	cvxopt.solvers.options['show_progress'] = False
	uSol = cvxopt.solvers.qp(cvxopt.matrix(H),cvxopt.matrix(f),cvxopt.matrix(A),cvxopt.matrix(b))
	uActual = uSol['x']

	print "uActual=",uActual
	
	# for deadlock
	'''if np.linalg.norm(statei-sphero.goalPose[:,i]) > 2*sphero.closeEnoughPose and np.linalg.norm(uActual) < sphero.deadlockThreInput:
		matTmp = np.matrix([[math.cos(sphero.controlTurnAng),-math.sin(sphero.controlTurnAng)],[math.sin(sphero.controlTurnAng),math.cos(sphero.controlTurnAng)]])
		uActual = uActual+sphero.kDelta*np.dot(matTmp,uNominal)
		print "deadlock!!!" '''
	
	return uActual
	
	




