#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Point32,Pose
import transforms3d as tf
import numpy as np

pi = 3.141592653589793238

#class to define vehicle position on a coordinate system at a certain heading
class Point:

    def __init__(self,inputX = 0,inputY = 0,inputHeading = 0):
        self.x = inputX
        self.y = inputY
        self.heading = inputHeading

        # class to define vehicle parameters
class AckermannVehicle:
    def __init__(self,inputLength = 1,inputMaximumSteeringAngle = 1,inputMaximumVelocity = 48*pi/180):
        self.length = inputLength
        self.maximumSteeringAngle = inputMaximumSteeringAngle
        self.maximumVelocity = inputMaximumVelocity
        self.minTurningRadius = self.length / math.tan(self.maximumSteeringAngle)

# class to define Pure pursuite controller parameters
class PPController:

    def __init__(self,inputLeadDistance,inputLength):
        self.leadDistance = inputLeadDistance
        self.length = inputLength #set the length for ppcontroller as the length of maximumSteeringAngle

        # compute the steering radius of ackerman vehicle of given parameters
    def compute_turning_radius(self, inputCurrent = Point(0,0,0) , inputGoal = Point(0,0,0)):
        global current
        global goal
        global alpha

        current = inputCurrent
        goal = inputGoal
        beta = math.atan2((goal.y - current.y),(goal.x-current.y)) # angle between line joining start-end and x axis
        temp = ((current.heading +  3.14))
        temp = (math.fmod(temp , 2*3.14)) - 3.14
        alpha = temp - beta #angle between current heading and the line joining start-end
        euclideanDistance = math.sqrt(math.pow((goal.x - current.x),2) + math.pow((goal.y-current.y),2))
        turningRadius = euclideanDistance / (2*math.sin(alpha)) #this outputs the turning radius

# compute the steering angle of ackermann vehicle of given paramters
    def compute_steering_angle(self):
        steeringAngle = 0.7*math.sin(alpha)*self.length

        return steeringAngle

# compute forward velocity relative to steering angle
    def compute_forward_velocity(self): #added a variable velocity based on Bijo's suggestion
        #forwardVelocity = mule.maximumVelocity * (1 - atan(abs(steeringAngle))/(pi/2));  //this specifies the forward velocity at a given steering angle
        forwardVelocity = 1

        return forwardVelocity
        
# this is a test line...
def XYZcallback(data):
    x = data.position.x
    y = data.position.y
    z = data.position.z

    euler = tf.euler.quat2euler([data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w])
    yaw = euler[2]
    # this is a second test line...
def command():
    global currentPoint
    global goalPoint

    mule = AckermannVehicle(10,60*pi/180,1)
    senaPurePursuit = PPController(0,mule.length)


    rospy.Subscriber("/agBOT/local/Pose", Pose, XYZcallback)
    pub = rospy.Publisher('/agBOT/ackermann_cmd', Point32, queue_size =10)
    rospy.init_node('ppcontroller', anonymous=True)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():


        threshold = 0.5
        euclideanError = 0

        while (euclideanError < threshold):
            goalPoint = Point()
            currentPoint = Point()

            goalPoint.x = int(input('Enter goX:'))
            goalPoint.y = int(input('Enter goY:'))
            goalPoint.heading = int(input('Enter goHeading:'))


            senaPurePursuit.compute_turning_radius(currentPoint, goalPoint)
            senaPurePursuit.compute_steering_angle()

            euclideanError = math.sqrt((math.pow((goalPoint.x-currentPoint.x),2) + math.pow((goalPoint.y-currentPoint.y),2)))


        # Recompute Euclidean error if euclideanErro !< threshold:
        euclideanError = math.sqrt((math.pow((goalPoint.x-currentPoint.x),2) + math.pow((goalPoint.y-currentPoint.y),2)))

        if (euclideanError > threshold):
            # Compute turningRadius , steeringAngle and velocity for current start and goal point:
            senaPurePursuit.compute_turning_radius(currentPoint, goalPoint)
            command = Point32()
            command.x = senaPurePursuit.compute_steering_angle()
            command.y = senaPurePursuit.compute_forward_velocity()


        pub.publish(command)
        rate.sleep()

    rospy.spin()

if __name__ == '__main__':
    command()
