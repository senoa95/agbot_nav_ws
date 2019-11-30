#!/usr/bin/env python

# Import libraries:
import rospy
import math
from geometry_msgs.msg import Point32,Pose
from utilities import Point, AckermannVehicle , PPController, DiffDriveVehicle
import transforms3d as tf
import numpy as np
import os
import rospkg
import time

rospack = rospkg.RosPack()


### Define constants:

# Define global variables:
global currentPos
currentPos = Point()
global file_name
# file_name = rospy.get_param("/file_name")
file_name = "waypoints_normalized_0.txt"

# Callback function for subscriber to Position and orientation topic:
def XYZcallback(data):

    global currentPos

    currentPos.x = data.position.x
    currentPos.y = data.position.y

    euler = tf.euler.quat2euler([data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w])
    #euler[1] = euler[1] - 1.57
    #euler[2] = euler[2] + 3.14
    currentPos.heading = euler[0]




# 1. Initialize function definition:
def initialize():

    global file_name
    # Create objects for AckermannVehicle and Pure Pursuit controller:
    mule = DiffDriveVehicle(0.455,0.0,0.195,3)
    cntrl = PPController(0,mule.length,mule.minTurningRadius,mule.maximumVelocity)

    cntrl.initialize(os.path.join(rospack.get_path("agbot_nav"),"src",file_name))

    return cntrl


# 2. Execute function definition:
def execute(cntrl):

    global currentPos

    # Setup the ROS publishers and subscribers:
    rospy.Subscriber("/pr2/local/Pose", Pose, XYZcallback)
    pub = rospy.Publisher('/pr2/cmd_vel', Point32, queue_size =10)
    pub_goal = rospy.Publisher('/current_goalpoint',Point32,queue_size=10)
    rospy.init_node('ppcontroller', anonymous=True)

    rate = rospy.Rate(10)

    # Initialize:
    # 1. Parameters:
    threshold = 0.5
    euclideanError = 0

    # 2. Points:
    goalPoint = cntrl.wpList[cntrl.currWpIdx]

    # 3. Commands:
    command = Point32()
    stationaryCommand = Point32()

    stationaryCommand.x = 0
    stationaryCommand.y = 0

    # Loop through as long as the node is not shutdown:
    while not rospy.is_shutdown():

        # Compute the new Euclidean error:
        current_goalPoint = Point32(goalPoint.x,goalPoint.y,0)
        # current_goalPoint = [str(goalPoint.x),str(goalPoint.y),'0']
        pub_goal.publish(current_goalPoint)

        euclideanError = math.sqrt((math.pow((goalPoint.x-currentPos.x),2) + math.pow((goalPoint.y-currentPos.y),2)))
   
        # Case #1:Vehicle is in the vicinity of current goal point (waypoint):
        if (euclideanError < threshold):

            # Make the AckermannVehicle stop where it is
            pub.publish(stationaryCommand)

            print (" Reached Waypoint # ", cntrl.currWpIdx +1)
            # Update goal Point to next point in the waypoint list:
            cntrl.currWpIdx +=1

            if cntrl.currWpIdx < cntrl.nPts:
                goalPoint = cntrl.wpList[cntrl.currWpIdx]

            else:

                print (" --- All Waypoints have been conquered! Mission Accomplished Mr Hunt !!! --- ")
                break


            print (" New goal is: ")
            print (goalPoint.x)
            print (goalPoint.y)
        print (" Euclidean Error = ", euclideanError , " meters")
        


        

        # Case #2:
        if (euclideanError > threshold):

            # Compute steering and velocity commands according to Dr L controller
            vel, delta = cntrl.compute_steering_vel_cmds(currentPos)

            command = Point32()
            command.x = vel
            command.y = delta
            
            # Publish the computed command:
            pub.publish(command)

            # Recompute the Euclidean error to see if its reducing:
            euclideanError = math.sqrt((math.pow((goalPoint.x-currentPos.x),2) + math.pow((goalPoint.y-currentPos.y),2)))


        rate.sleep()

    rospy.spin()

if __name__ == '__main__':

    # Step 1: Initialize the Controller by reading in the list of waypoints:
    cntrl = initialize()

    # Step 2: Execute the controller in a closed loop manner
    execute(cntrl)
