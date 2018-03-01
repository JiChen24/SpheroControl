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
from collisionFreeDecentralized import *


odometry = None
init_odom = None
theta_cal=0

wallRange = 5.0
MAP = numpy.matrix([[-wallRange,-wallRange,wallRange,-wallRange],[wallRange,-wallRange,wallRange,wallRange],[wallRange,wallRange,-wallRange,wallRange],[-wallRange,wallRange,-wallRange,-wallRange]])

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
    sphero_name1 = str(sys.argv[1])
    sphero_name2 = str(sys.argv[2])
    sphero_name3 = str(sys.argv[3])
    sphero_name4 = str(sys.argv[4])

    Num = int(sys.argv[5])

    obj = swarmParams.sysParams(8)

    odd = -1

    waypoints1=odd*numpy.array([ [-1.5,0.0] ])
    waypoints2=odd*numpy.array([ [0.0,-1.5] ])
    waypoints3=odd*numpy.array([ [1.5,0.0] ])
    waypoints4=odd*numpy.array([ [0.0,1.5] ])

    waypoints = [waypoints1,waypoints2,waypoints3,waypoints4]

    neighborRadius = 10.0

    size_waypoints=waypoints[0].shape #[m,n] length and width of waypoints
    num_waypoints=size_waypoints[0] #first entry is length (width is 2-(x,y))
    vel_mag=100

    rospy.init_node(sphero_name1+"_test_vel")

    # publish to topics
    pub1 = rospy.Publisher(sphero_name1+'/cmd_vel', geometry_msgs.msg.Twist, queue_size=1, latch=True)
    color_pub1 = rospy.Publisher(sphero_name1 + '/set_color', ColorRGBA, queue_size=1, latch=True)

    pub2 = rospy.Publisher(sphero_name2+'/cmd_vel', geometry_msgs.msg.Twist, queue_size=1, latch=True)
    color_pub2 = rospy.Publisher(sphero_name2 + '/set_color', ColorRGBA, queue_size=1, latch=True)

    pub3 = rospy.Publisher(sphero_name3+'/cmd_vel', geometry_msgs.msg.Twist, queue_size=1, latch=True)
    color_pub3 = rospy.Publisher(sphero_name3 + '/set_color', ColorRGBA, queue_size=1, latch=True)

    pub4 = rospy.Publisher(sphero_name4+'/cmd_vel', geometry_msgs.msg.Twist, queue_size=1, latch=True)
    color_pub4 = rospy.Publisher(sphero_name4 + '/set_color', ColorRGBA, queue_size=1, latch=True)

    pub = [pub1,pub2,pub3,pub4]
    color_pub = [color_pub1,color_pub2,color_pub3,color_pub4]

    grn_color = std_msgs.msg.ColorRGBA(g=255)
    blu_color = std_msgs.msg.ColorRGBA(b=255)
    col_msg = grn_color
    flash_risingEdge = True

    rate = rospy.Rate(10) # set publish rate

    # subscribe to odometry data
    rospy.Subscriber(sphero_name1+'/odom', Odometry, callback=save_odometry)
    
    #Initialize the sphero as a vicon tracked object
    a1 = vt.ViconTrackerPoseHandler(None, None, "",51017, 'Sphero1')
    a2 = vt.ViconTrackerPoseHandler(None, None, "",51018, 'Sphero2')
    a3 = vt.ViconTrackerPoseHandler(None, None, "",51019, 'Sphero3')
    a4 = vt.ViconTrackerPoseHandler(None, None, "",51020, 'Sphero4')

    aVicon = [a1,a2,a3,a4]

    b1 = vt.ViconTrackerPoseHandler(None, None, "",51021, 'Sphero5')
    b2 = vt.ViconTrackerPoseHandler(None, None, "",51022, 'Sphero6')
    b3 = vt.ViconTrackerPoseHandler(None, None, "",51023, 'Sphero7')
    b4 = vt.ViconTrackerPoseHandler(None, None, "",51024, 'Sphero8')

    bVicon = [b1,b2,b3,b4]

    obj.initPose()
    obj.goalPose()
    obj.neighborRadius()

    for i in range(Num):
        obj.goalPose[0,4+i] = waypoints[i][0,0]
        obj.goalPose[1,4+i] = waypoints[i][0,1]

    #Calibrate sphero frame
    
    sphero_theta = [0.0,0.0,0.0,0.0]

    for i in range(Num):

    	rate.sleep()
        startCalib = time.time()
        init_calib_loc = bVicon[i].getPose()
        print init_calib_loc[0]
        print init_calib_loc[1]
        while (time.time() - startCalib < 2):
            vel_msg = geometry_msgs.msg.Twist(geometry_msgs.msg.Vector3(150,0,0), \
                                              geometry_msgs.msg.Vector3(0,0,0))
            pub[i].publish(vel_msg)
            rate.sleep()

        vel_msg = geometry_msgs.msg.Twist(geometry_msgs.msg.Vector3(0,0,0), \
                                              geometry_msgs.msg.Vector3(0,0,0))

        pub[i].publish(vel_msg)

        fin_calib_loc = bVicon[i].getPose()
        print fin_calib_loc[0]
        print fin_calib_loc[1]
        calibVect = numpy.array([fin_calib_loc[0]-init_calib_loc[0],fin_calib_loc[1]-init_calib_loc[1]])
        print calibVect[0]
        print calibVect[1]
        calib_theta = numpy.arctan2(calibVect[1],calibVect[0])
        sphero_theta[i] = calib_theta
        print "Sphero Theta is "+str(180/math.pi*sphero_theta[i])+"\n"

#loop through the goal points and move to all of them
for ind in range(0,num_waypoints):
#index of first goal is 0, last is num_waypoints-1
    #extract the current goal location
    loc_goal=[[],[],[],[]]
    loc_init=[[],[],[],[]]
    vect2goal=[[],[],[],[]]
    dist2goal=[[],[],[],[]]
    rate.sleep()
    for i in range(Num):
        loc_goal[i].append(waypoints[i][ind][0])
        loc_goal[i].append(waypoints[i][ind][1])
        loc_init[i]=bVicon[i].getPose()
        vect2goal[i] = numpy.array([(loc_goal[i][0]-loc_init[i][0]),(loc_goal[i][1]-loc_init[i][1])])
        dist2goal[i] = numpy.sqrt(vect2goal[i].dot(vect2goal[i]))

    dist2goalAll = sum(dist2goal)

    start = time.time()

    while (dist2goalAll > 0.8): # not at goal point

        for i in range(Num):
            loc1 = a1.getPose()
            loc2 = a2.getPose()
            loc3 = a3.getPose()
            loc4 = a4.getPose()
            locAnother = [loc1,loc2,loc3,loc4]

            loc5 = b1.getPose()
            loc6 = b2.getPose()
            loc7 = b3.getPose()
            loc8 = b4.getPose()
            loc = [loc5,loc6,loc7,loc8]

            poseAll = numpy.array([[loc1[0],loc2[0],loc3[0], loc4[0], loc5[0], loc6[0],loc7[0], loc8[0]],[loc1[1],loc2[1],loc3[1], loc4[1], loc5[1], loc6[1], loc7[1], loc8[1]]])
            
            statesNeighbor = numpy.delete(poseAll,numpy.s_[i+4:i+5],axis=1)
            poseAll = numpy.matrix(poseAll)
            statesNeighbor = numpy.matrix(statesNeighbor)

            obj.getCurrentPose(poseAll)

            indNeighborOrig = numpy.array([0,1,2,3,4,5,6,7])
            indNeighbor = numpy.delete(indNeighborOrig,numpy.s_[i+4:i+5],axis=0)
            indNeighbor = numpy.matrix(indNeighbor)

            vect2goal[i] = numpy.array([(loc_goal[i][0]-loc[i][0]),(loc_goal[i][1]-loc[i][1])])
            dist2goal[i] =  numpy.sqrt(vect2goal[i].dot(vect2goal[i]))

            print "vect2goal[i]="
            print vect2goal[i]
            #Calculate the required vel_msg to reach the goal point from the current location in the vicon frame

            if dist2goal[i] > 0.1:

                v_vect_V_actual = actualController(poseAll[:,i+4],i+4,statesNeighbor,indNeighbor,obj,MAP)
                print "actual_vel="
                print v_vect_V_actual

                v_vect_V_actual = numpy.array([[v_vect_V_actual[0]],[v_vect_V_actual[1]]])

                thetaNew = numpy.arctan2((v_vect_V_actual[1]),(v_vect_V_actual[0]))
                v_x_V_new = vel_mag*math.cos(thetaNew)
                v_y_V_new = vel_mag*math.sin(thetaNew)
                v_vect_V_update = numpy.array([[v_x_V_new],[v_y_V_new]])

                print "actual_vel new is"
                print v_vect_V_update

                print "Position is"
                print loc[i]

                R_SV=numpy.array([ [math.cos(sphero_theta[i]),math.sin(sphero_theta[i])],[-math.sin(sphero_theta[i]), math.cos(sphero_theta[i])] ])
        	   # convert global frame to sphero frame
                v_vect_S=numpy.dot(R_SV,v_vect_V_update)
                v_x_S=v_vect_S[0]
                v_y_S=v_vect_S[1]
                #print v_vect_S

                #Define the vel_msg command to send to the sphero

                #vel_msg = geometry_msgs.msg.Twist(geometry_msgs.msg.Vector3(v_vect_S[0],v_vect_S[1],0), \
                                              #geometry_msgs.msg.Vector3(0,0,0))
                vel_msg = geometry_msgs.msg.Twist(geometry_msgs.msg.Vector3(v_vect_S[0],v_vect_S[1],0), \
                                              	geometry_msgs.msg.Vector3(0,0,0))

            else:
                vel_msg = geometry_msgs.msg.Twist(geometry_msgs.msg.Vector3(0,0,0), \
                                                geometry_msgs.msg.Vector3(0,0,0))

            vel_msg_zero = geometry_msgs.msg.Twist(geometry_msgs.msg.Vector3(0,0,0), \
                                          	geometry_msgs.msg.Vector3(0,0,0))

            #rospy.loginfo("Velocity: {0}".format(vel_msg))
            '''if flash_risingEdge:
            	col_msg = grn_color
            	flash_risingEdge = False
            else:
            	col_msg = blu_color
            	flash_risingEdge = True'''

            pub[i].publish(vel_msg)
            color_pub[i].publish(col_msg)
            rate.sleep()

        dist2goalAll = sum(dist2goal)
     

rospy.loginfo("Last Odometry: {0}".format(odometry))
rospy.loginfo("Last Odometry: {0}".format(odometry))
print loc

for i in range(Num):
	color_pub[i].publish(grn_color)
