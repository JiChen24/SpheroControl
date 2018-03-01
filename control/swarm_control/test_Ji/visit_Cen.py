#!/usr/bin/env python

import rospy
import geometry_msgs.msg
import argparse
import random
import std_msgs
from std_msgs.msg import ColorRGBA, Float32, Bool
from nav_msgs.msg import Odometry
import time
import math
import numpy
import ViconTrackerPoseHandler as vt
import sys

# collision avoidance
import swarmParams
from collisionFreeCentralized import *
import pathPlanning
import Sync2



odometry = None
init_odom = None
theta_cal=0

MAP = pathPlanning.createMap()
nodes = pathPlanning.createNodes()

def save_odometry(data):
    global odometry

    # save initial
    if odometry is None:
        global init_odom
        init_odom = data.pose.pose

    odometry =data.pose.pose

#Define the waypoints to be visited and the speed at which to visit them
    # magnitude min 5 for lab ground
    # magnitude min 30 for lab carpet
    # magnitude min 150 for lab carpet with vicon tracker cup
#waypoints=numpy.array([ [1,1], [-1,1] ])


#loc_goal=[]
#loc_goal.append(1)
#loc_goal.append(1)

if __name__=="__main__":
    #parser = argparse.ArgumentParser(description="Test Sphero location")
    #args, unknown = parser.parse_known_args()
    viconStartNumHere = int(sys.argv[-3])
    viconBegin = int(sys.argv[-2])
    viconEnd = int(sys.argv[-1])
    Diff = viconStartNumHere-1
    numAll = viconEnd-viconBegin+1

    Num = len(sys.argv)-4

    sphero_name = [[]]*Num
    for i in range(Num):
        sphero_name[i] = 'sphero_'+str(sys.argv[i+1])

    obj = swarmParams.sysParams(numAll)

    M1 = numpy.matrix([2,1,8,1,5,1,3])
    M2 = numpy.matrix([2,1,8,1,6,1,3])
    M3 = numpy.matrix([2,1,8,1,7,1,4])
    '''M1 = numpy.matrix([2,1])
    M2 = numpy.matrix([2,1])
    M3 = numpy.matrix([2,1])'''
    autoMat = (M1,M2,M3)

    S1 = numpy.matrix([[0,0,0,0,0,0,0],[1,1,0,0,1,1,1],[1,1,0,0,1,1,1]])
    S2 = numpy.matrix([[1,1,0,0,1,1,1],[0,0,0,0,0,0,0],[1,1,0,0,1,1,1]])
    S3 = numpy.matrix([[1,1,0,0,1,1,1],[1,1,0,0,1,1,1],[0,0,0,0,0,0,0]])

    '''S1 = numpy.matrix([[1,1],[1,1],[1,1]])
    S2 = numpy.matrix([[1,1],[1,1],[1,1]])
    S3 = numpy.matrix([[1,1],[1,1],[1,1]])'''
    syncMat=(S1,S2,S3)

    numOfGroups = 3
    loopingIndex = 2

    regionCenters = numpy.matrix([[0,1.8,-0.3,1.1,-0.3,0.7,2.1,-2.2],[0.2,0.2,2.6,2.6,-2.2,-2.2,-2.2,0.2]])
    tol = 0.1
    offsetVector = numpy.matrix([[0.6+tol,0.5+tol,0.45+tol,0.45+tol,0.3+tol,0.3+tol,0.3+tol,0.5+tol],[0.6+tol,0.6+tol,0.5+tol,0.5+tol,0.5+tol,0.5+tol,0.5+tol,0.7+tol]])
    
    (numOfStates,memory,goalPoseForSync,offsetGoal,advance) = Sync2.initValues(autoMat,syncMat,numAll,numOfGroups,regionCenters,offsetVector)

    #print "goalPoseInit=",goalPoseForSync

    '''odd = -1

    theta0 = math.pi/6
    radius0 = 2.0

    waypoints = [[]]*Num
    for i in range(Num):
        waypoints[i] = odd*radius0*numpy.array([ [math.cos(i*2*math.pi/Num+theta0),math.sin(i*2*math.pi/Num+theta0)] ])'''

    waypoints = [[]]*Num

    odd = -1
    if odd == 1:
    	waypoints[0] = numpy.array([[0.6,2.5]])
    	waypoints[1] = numpy.array([[-2.2,0.0]])
    	waypoints[2] = numpy.array([[1.2,-2.5]])
        #waypoints[3] = numpy.array([[1.2,-2.5]])
        #waypoints[4] = numpy.array([[1.2,-2.5]])
        #waypoints[5] = numpy.array([[1.2,-2.5]])
    else:
    	waypoints[0] = numpy.array([[-0.8,-2.5]])
    	waypoints[1] = numpy.array([[1.2,-2.5]])
    	waypoints[2] = numpy.array([[-0.6,2.5]])
        #waypoints[3] = numpy.array([[1.2,-2.5]])
        #waypoints[4] = numpy.array([[1.2,-2.5]])
        #waypoints[5] = numpy.array([[1.2,-2.5]])



    # neighborRadius = 10.0

    size_waypoints=waypoints[0].shape #[m,n] length and width of waypoints
    num_waypoints=size_waypoints[0] #first entry is length (width is 2-(x,y))
    vel_mag=80

    rospy.init_node(sphero_name[0]+"_test_vel")

    # publish to topics
    pub = [[]]*Num
    color_pub = [[]]*Num

    for i in range(Num):
        pub[i] = rospy.Publisher(sphero_name[i]+'/cmd_vel', geometry_msgs.msg.Twist, queue_size=1, latch=True)
        color_pub[i] = rospy.Publisher(sphero_name[i] + '/set_color', ColorRGBA, queue_size=1, latch=True)


    grn_color = std_msgs.msg.ColorRGBA(g=255)
    blu_color = std_msgs.msg.ColorRGBA(b=255)
    col_msg = grn_color
    flash_risingEdge = True

    rate = rospy.Rate(8) # set publish rate

    # subscribe to odometry data
    rospy.Subscriber(sphero_name[0]+'/odom', Odometry, callback=save_odometry)
    
    #Initialize the sphero as a vicon tracked object
    aVicon = [[]]*numAll
    bVicon = [[]]*Num
    for i in range(numAll):
        aVicon[i] = vt.ViconTrackerPoseHandler(None, None, "",viconBegin+i, 'Sphero'+str(i+1))

    for i in range(Num):
        bVicon[i] = vt.ViconTrackerPoseHandler(None, None, "",viconBegin+Diff+i, 'Sphero'+str(i+1+Diff))

    obj.initPose()
    obj.goalPose()
    obj.neighborRadius()

    #print "Diff=",Diff
    #print obj.goalPose
    #print waypoints

    for i in range(Num):
        print i
        obj.goalPose[0,Diff+i] = waypoints[i][0,0]
        obj.goalPose[1,Diff+i] = waypoints[i][0,1]

    #Calibrate sphero frame
    
    sphero_theta = [0.0]*Num

    for i in range(Num):

    	#rate.sleep()
        startCalib = time.time()
        init_calib_loc = bVicon[i].getPose()
        #print init_calib_loc[0]
        #print init_calib_loc[1]
        while (time.time() - startCalib < 1):
            vel_msg = geometry_msgs.msg.Twist(geometry_msgs.msg.Vector3(100,0,0), \
                                              geometry_msgs.msg.Vector3(0,0,0))
            pub[i].publish(vel_msg)
            rate.sleep()

        vel_msg = geometry_msgs.msg.Twist(geometry_msgs.msg.Vector3(0,0,0), \
                                              geometry_msgs.msg.Vector3(0,0,0))

        pub[i].publish(vel_msg)
        rate.sleep()

        fin_calib_loc = bVicon[i].getPose()
        #print fin_calib_loc[0]
        #print fin_calib_loc[1]
        calibVect = numpy.array([fin_calib_loc[0]-init_calib_loc[0],fin_calib_loc[1]-init_calib_loc[1]])
        #print calibVect[0]
        #print calibVect[1]
        calib_theta = numpy.arctan2(calibVect[1],calibVect[0])
        sphero_theta[i] = calib_theta
        #print "Sphero Theta is "+str(180/math.pi*sphero_theta[i])+"\n"

locAll = [[]]*numAll
poseAll = numpy.zeros([2,numAll])
waypoints_Tmp = numpy.zeros([2,Num])

#loop through the goal points and move to all of them
for ind in range(0,num_waypoints):
#index of first goal is 0, last is num_waypoints-1
    #extract the current goal location
    loc_goal=[[]]*Num
    loc_init=[[]]*Num
    vect2goal=[[]]*Num
    dist2goal=[[]]*Num
    rate.sleep()
    for i in range(Num):
        #loc_goal[i].append(waypoints[i][ind][0])
        #loc_goal[i].append(waypoints[i][ind][1])

        loc_goal[i]=[waypoints[i][ind][0],waypoints[i][ind][1]]
        loc_init[i]=bVicon[i].getPose()
        vect2goal[i] = numpy.array([(loc_goal[i][0]-loc_init[i][0]),(loc_goal[i][1]-loc_init[i][1])])
        dist2goal[i] = numpy.sqrt(vect2goal[i].dot(vect2goal[i]))

    #dist2goalAll = sum(dist2goal)

    start = time.time()

    while (time.time()-start<90): # not at goal point

    	(memoryNew,goalPoseForSyncNew,offsetGoalNew,advanceNew) = Sync2.isReady(autoMat,syncMat,numAll,numOfGroups,numOfStates,memory,goalPoseForSync,offsetGoal,\
            															poseAll,loopingIndex,advance,regionCenters,offsetVector)

    	memory = memoryNew
    	goalPoseForSync = goalPoseForSyncNew
    	offsetGoal = offsetGoalNew
    	advance = advanceNew

    	print "goalPoseForSync=",goalPoseForSync

    	rate.sleep()

    	for j in range(Num):
            	loc_goal[j][0] = goalPoseForSync[0,j+Diff]
            	loc_goal[j][1] = goalPoseForSync[1,j+Diff]

        for j in range(numAll):
            locAll[j] = aVicon[j].getPose()
            poseAll[0,j] = locAll[j][0]
            poseAll[1,j] = locAll[j][1]

        poseAll = numpy.matrix(poseAll)
        obj.getCurrentPose(poseAll)

        print "poseAll=",poseAll

        for j in range(Num):
            waypoints_TmpTmp = pathPlanning.findPath(MAP,nodes,poseAll[:,j+Diff],numpy.array([[loc_goal[j][0]],[loc_goal[j][1]]]))
            waypoints_Tmp[0,j] = waypoints_TmpTmp[0,0]
            waypoints_Tmp[1,j] = waypoints_TmpTmp[1,0]
            obj.goalPose[:,j+Diff] = numpy.matrix(waypoints_Tmp[:,j])

        print 'goalPose is', obj.goalPose

        try:
            v_vect_V_actual = actualController(poseAll,obj,MAP)
            v_vect_V_actual = np.matrix(v_vect_V_actual)
        except ValueError:
            v_vect_V_actual = np.zeros([2*numAll,1])
            v_vect_V_actual = np.matrix(v_vect_V_actual)

        for j in range(Num):
            thetaNew = numpy.arctan2(v_vect_V_actual[2*j+2*Diff+1,0],v_vect_V_actual[2*j+2*Diff,0])
            v_x_V_new = vel_mag*math.cos(thetaNew)
            v_y_V_new = vel_mag*math.sin(thetaNew)
            v_vect_V_update = numpy.array([[v_x_V_new],[v_y_V_new]])

            R_SV=numpy.array([ [math.cos(sphero_theta[j]),math.sin(sphero_theta[j])],[-math.sin(sphero_theta[j]), math.cos(sphero_theta[j])] ])
            v_vect_S=numpy.dot(R_SV,v_vect_V_update)

            v_x_S=v_vect_S[0]
            v_y_S=v_vect_S[1]

            #print "v_x_S=",v_x_S
            #print "v_y_S=",v_y_S

            vel_msg = geometry_msgs.msg.Twist(geometry_msgs.msg.Vector3(v_x_S,v_y_S,0), \
                                                geometry_msgs.msg.Vector3(0,0,0))
            vel_msg_zero = geometry_msgs.msg.Twist(geometry_msgs.msg.Vector3(0,0,0), \
                                                geometry_msgs.msg.Vector3(0,0,0))


            if flash_risingEdge:
                col_msg = grn_color
                flash_risingEdge = False
            else:
                col_msg = blu_color
                flash_risingEdge = True


            pub[j].publish(vel_msg)
            color_pub[j].publish(col_msg)
            rate.sleep()

     

rospy.loginfo("Last Odometry: {0}".format(odometry))
rospy.loginfo("Last Odometry: {0}".format(odometry))
#print locAll

for j in range(Num):
	color_pub[i].publish(grn_color)
