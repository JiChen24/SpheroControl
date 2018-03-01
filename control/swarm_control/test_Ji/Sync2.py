import numpy as np
import math

def initValues(autoMat,syncMat,numOfAgents,numOfGroups,regionCenters,offsetVector):
	numOfStates = autoMat[0].shape[1]
	memory = np.zeros(numOfAgents)
	memory = np.matrix(memory)
	goalPose = np.zeros([2,numOfAgents])
	goalPose = np.matrix(goalPose)
	offsetGoal = np.zeros([2,numOfAgents])
	offsetGoal = np.matrix(offsetGoal)
	advance = [[]]*numOfGroups
	for i in range(numOfGroups):
		advance[i] = np.matrix(np.zeros([numOfGroups,1]))

	for i in range(numOfAgents):

		#print "memory[0,",i,"]=",memory[0,i]
		xyPose = autoMat[(i+1)%numOfGroups][0,memory[0,i]]-1

		#print "xyPose",(i+1)%numOfGroups,"=",xyPose

		goalPose[:,i] = regionCenters[:,xyPose]
		offsetGoal[:,i] = offsetVector[:,xyPose]

	return (numOfStates,memory,goalPose,offsetGoal,advance)

def isReady(autoMat,syncMat,numOfAgents,numOfGroups,numOfStates,memory,goalPose,offsetGoal,currentPose,loopingIndex,advance,regionCenters,offsetVector):
	# lists of 1-by-N matrices
	#numOfGroups = len(autoMat)
	groupCaughtUp = np.ones(numOfGroups)
	groupCaughtUp = np.matrix(groupCaughtUp)
	for i in range(numOfAgents):
		if goalPose[0,i]-offsetGoal[0,i]>currentPose[0,i] or goalPose[0,i]+offsetGoal[0,i]<currentPose[0,i] or goalPose[1,i]-offsetGoal[1,i]>currentPose[1,i] or goalPose[1,i]+offsetGoal[1,i]<currentPose[1,i]:
			groupCaughtUp[0,(i+1)%numOfGroups] = 0

	groupReady = np.ones(numOfGroups)
	groupReady = np.matrix(groupReady)

	#print "groupCaughtUp=",groupCaughtUp

	for i in range(numOfGroups):
		if groupCaughtUp[0,i] == 1:
			index = i-1
			if index < 0:
				index = numOfGroups-1

			syncVector = syncMat[i][:,memory[0,index]]

			for j in range(numOfGroups):
				if i == j:
					# continue
					print "."
				else:
					if syncVector[j,0] == 1:
						if advance[i][j,0] != 1:
							#print "groupCaughtUp[0,",j,"]=",groupCaughtUp[0,j]
							if groupCaughtUp[0,j] == 1:
								index2 = j-1
								if index2 < 0:
									index2 = numOfGroups-1

								syncVector_j = syncMat[j][:,memory[0,index2]]

								#print "syncVector_j=",syncVector_j
								#print "advance=",advance,"A"

								if syncVector_j[i,0]==1 and advance[j][i,0]==0:
									#print "advance=",advance,"B1"
									#print "i=",i
									#print "j=",j
									#print "advance[i][j,0]=",advance[i][j,0]
									#print "advance[j][i,0]=",advance[j][i,0]
									advance[i][j,0] = 1
									advance[j][i,0] = 1
									#print "advance=",advance,"B2"
								else:
									groupReady[0,i] = 0
									#print "groupReady[0,",i,"]=",groupReady[0,i],"  j=",j,"C"
							else:
								groupReady[0,i] = 0
								#print "groupReady[0,",i,"]=",groupReady[0,i],"  j=",j,"D"
						else:
							groupReady[0,i] = groupReady[0,i]
							#print "groupReady[0,",i,"]=",groupReady[0,i],"  j=",j,"E"
					else:
						groupReady[0,i] = groupReady[0,i]
						#print "groupReady[0,",i,"]=",groupReady[0,i],"  j=",j,"F"

		else:
			#print "group is not Ready for GROUP", i+1
			groupReady[0,i] = 0

	#print "groupReady=", groupReady

	for i in range(numOfAgents):
		groupId = (i+1)%numOfGroups
		if groupReady[0,groupId] == 1:
			if memory[0,i] == numOfStates-1:
				memory[0,i] = loopingIndex-1
			else:
				memory[0,i] = memory[0,i]+1

			#nextPoseOld = nextPose

			advance[groupId] = np.matrix(np.zeros([numOfGroups,1]))
			nextPose = autoMat[groupId][0,memory[0,i]]-1

			#print "nextPose=",nextPose

			goalPose[:,i] = regionCenters[:,nextPose]
			offsetGoal[:,i] = offsetVector[:,nextPose]
		else:
			#print "Do NOT change the goalPose"
			print "."
		    

	return (memory,goalPose,offsetGoal,advance)


