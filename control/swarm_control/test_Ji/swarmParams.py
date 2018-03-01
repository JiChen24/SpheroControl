import numpy as np
#import scipy as sp
import math


class sysParams(object):
	
	def __init__(self,n):
		self.n = n
		self.inputLim = np.ones(n)*0.1
		self.kp = 10.0
		self.Ds = 0.35
		self.DsWall = 0.25
		self.gamma = 8
		self.closeEnoughPose = 0.1
		
		# deadlock
		self.deadlockThreInput = 0.08
		self.kDelta = 2.0
		self.controlTurnAng = 3.0*math.pi/4
		
		
		
		
	# get initial poses
	def initPose(self):
		self.theta0 = 0
		self.radius = 1.5
		self.initPose = np.zeros([2,self.n])
		
		# self.initPose = pose
	
	# get goal poses
	def goalPose(self):
		self.goalPose = np.zeros([2,self.n])
		
		# self.goalPose = pose
		
	# get neighborhood radius
	def neighborRadius(self):
		self.Dneighbor = np.zeros(self.n)
		for i in xrange(self.n):
			betaMax = max(self.inputLim)
			self.Dneighbor[i] = self.Ds+((self.inputLim[i]+betaMax)/self.gamma)**(1.0/3)
			
		#return self
		
	# get current positions
	def getCurrentPose(self,pose):
		self.currentPose = pose
		#return self
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
