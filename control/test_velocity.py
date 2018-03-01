#!/usr/bin/env python

import rospy
import geometry_msgs.msg
import argparse
import random
from nav_msgs.msg import Odometry
import time
import sys

odometry = None
init_odom = None

def save_odometry(data):
    global odometry

    # save initial
    if odometry is None:
        global init_odom
        init_odom = data.pose.pose

    odometry =data.pose.pose

if __name__=="__main__":
    #parser = argparse.ArgumentParser(description="Test Sphero location")
    #args, unknown = parser.parse_known_args()
    print str(sys.argv[1])
    sphero = str(sys.argv[1])
    rospy.init_node(sphero+"test_vel")

    # publish to topics
    pub = rospy.Publisher(sphero +'/cmd_vel', geometry_msgs.msg.Twist, queue_size=10, latch=True)
    rate = rospy.Rate(10) # set publish rate

    # subscribe to odometry data
    rospy.Subscriber(sphero+'/odom', Odometry, callback=save_odometry)

    # min 5 for lab ground
    # min 30 for lab carpet
    vel_msg = geometry_msgs.msg.Twist(geometry_msgs.msg.Vector3(-90,0,0), \
                                      geometry_msgs.msg.Vector3(0,0,0))

    vel_msg_zero = geometry_msgs.msg.Twist(geometry_msgs.msg.Vector3(0,0,0), \
                                      geometry_msgs.msg.Vector3(0,0,0))

    rospy.loginfo("Velocity: {0}".format(vel_msg))
    start = time.time()
    while time.time()-start < 2: # not rospy.is_shutdown():
        pub.publish(vel_msg)
        rate.sleep()
    start = time.time()

    rospy.loginfo("Init Odometry: {0}".format(init_odom))
    rospy.loginfo("Odometry: {0}".format(odometry))

    time.sleep(1.5)
    rospy.loginfo("Last Odometry: {0}".format(odometry))

