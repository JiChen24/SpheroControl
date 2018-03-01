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
from collisionFreeDecentralized_test import *
import pathPlanning
import Sync2_test



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


    odd = -1

    theta0 = math.pi/6
    radius0 = 2.0

    waypoints = [[]]*Num
    for i in range(Num):
        waypoints[i] = odd*radius0*numpy.array([ [math.cos(i*2*math.pi/Num+theta0),math.sin(i*2*math.pi/Num+theta0)] ])


    # neighborRadius = 10.0

    size_waypoints=waypoints[0].shape #[m,n] length and width of waypoints
    num_waypoints=size_waypoints[0] #first entry is length (width is 2-(x,y))
    vel_mag=75

    rospy.init_node(sphero_name[0]+"_test_vel")

    # publish to topics
    pub = [[]]*Num
    color_pub = [[]]*Num

    for i in range(Num):
        pub[i] = rospy.Publisher(sphero_name[i]+'/cmd_vel', geometry_msgs.msg.Twist, queue_size=1, latch=True)
        color_pub[i] = rospy.Publisher(sphero_name[i] + '/set_color', ColorRGBA, queue_size=1, latch=True)


    grn_color = std_msgs.msg.ColorRGBA(g=255)
    blu_color = std_msgs.msg.ColorRGBA(b=255)
    groupColor = [std_msgs.msg.ColorRGBA(g=30),std_msgs.msg.ColorRGBA(r=30),std_msgs.msg.ColorRGBA(b=30)]
    col_msg = grn_color
    flash_risingEdge = True

    rate = rospy.Rate(6) # set publish rate

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

    print "Diff=",Diff
    print obj.goalPose
    print waypoints

    for i in range(Num):
        print i
        obj.goalPose[0,Diff+i] = waypoints[i][0,0]
        obj.goalPose[1,Diff+i] = waypoints[i][0,1]

    #Calibrate sphero frame
    
    sphero_theta = [0.0]*Num

    for i in range(Num):

    	rate.sleep()
        
        vel_msg = geometry_msgs.msg.Twist(geometry_msgs.msg.Vector3(185,0,0), \
                                              geometry_msgs.msg.Vector3(0,0,0))

        init_calib_loc = bVicon[i].getPose()

        startCalib = time.time()
        #print init_calib_loc[0]
        #print init_calib_loc[1]
        while (time.time() - startCalib < 2):
            #startT = time.time()

            pub[i].publish(vel_msg)
            #rate.sleep()

        vel_msg = geometry_msgs.msg.Twist(geometry_msgs.msg.Vector3(0,0,0), \
                                              geometry_msgs.msg.Vector3(0,0,0))
        T = time.time()-startCalib

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

        dis = (calibVect.dot(calibVect))**0.5

        print "init_calib_loc=", init_calib_loc
        print "fin_calib_loc=", fin_calib_loc

        print "time is ", T
        print "distance is", dis

        vel_ave = dis/T

        print "velocity is ", vel_ave

