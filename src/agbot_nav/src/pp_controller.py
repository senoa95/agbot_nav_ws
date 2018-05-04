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

# Define a global variable to keep track of the current position of Vehicle
global currentPoint
currentPoint = Point()

reachedGoal = False

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
        self.turningRadius = 1

        # compute the steering radius of ackerman vehicle of given parameters
    def compute_turning_radius(self, inputCurrent = Point(0,0,0) , inputGoal = Point(0,0,0)):

        current = inputCurrent
        goal = inputGoal
        beta = math.atan2((goal.y - current.y),(goal.x-current.y)) # angle between line joining start-end and x axis
        temp = ((current.heading +  3.14))
        temp = (math.fmod(temp , 2*3.14)) - 3.14
        alpha = temp - beta #angle between current heading and the line joining start-end
        L_a = math.sqrt(math.pow((goal.x - current.x),2) + math.pow((goal.y-current.y),2))
        self.turningRadius = L_a / (2*math.sin(alpha)) #this outputs the turning radius

# compute the steering angle of ackermann vehicle of given paramters
    def compute_steering_angle(self):

        # Steering angle command from Pure pursuit paper:
        # Steering angle = atan(L/R)
        steeringAngle = math.atan(self.length / self.turningRadius)

        # @senoa95: I'm commenting out this steering angle command. I did not understand this one.
        #steeringAngle = 0.7*math.sin(alpha)*self.length

        return steeringAngle

# compute forward velocity relative to steering angle
    def compute_forward_velocity(self): #added a variable velocity based on Bijo's suggestion
        #forwardVelocity = mule.maximumVelocity * (1 - atan(abs(steeringAngle))/(pi/2));  //this specifies the forward velocity at a given steering angle
        forwardVelocity = 0.4

        return forwardVelocity

# this is a test line...
def XYZcallback(data):

    global currentPoint

    currentPoint.x = data.position.x
    currentPoint.y = data.position.y

    euler = tf.euler.quat2euler([data.orientation.x,data.orientation.y,data.orientation.z,data.orientation.w])
    euler[1] = euler[1] - 1.57
    euler[2] = euler[2] + 3.14
    currentPoint.heading = euler[2]

def command():

    global currentPoint
    global goalPoint

    # Create objects for AckermannVehicle and Pure Pursuit controller:
    mule = AckermannVehicle(2.5772,60*pi/180,1)
    senaPurePursuit = PPController(0,mule.length)


    rospy.Subscriber("/agBOT/local/Pose", Pose, XYZcallback)
    pub = rospy.Publisher('/agBOT/ackermann_cmd', Point32, queue_size =10)
    rospy.init_node('ppcontroller', anonymous=True)

    rate = rospy.Rate(10)

    # Initialize:
    # 1. Parameters:
    threshold = 0.5
    euclideanError = 0

    # 2. Points:
    goalPoint = Point()
    currentPoint = Point()

    # 3. Commands:
    command = Point32()
    stationaryCommand = Point32()

    stationaryCommand.x = 0
    stationaryCommand.y = 0

    # Loop through as long as the node is not shutdown:
    while not rospy.is_shutdown():

        # Update the current Point:

        # Case #1:Vehicle is in the vicinity of current goal point (waypoint):
        if (euclideanError < threshold):

            reachedGoal = True

            # Make the AckermannVehicle stop where it is
            pub.publish(stationaryCommand)

            # Acquire the new goal point from the user:
            goalPoint.x = int(input('Enter goX:'))
            goalPoint.y = int(input('Enter goY:'))
            goalPoint.heading = int(input('Enter goHeading:'))

            # Recompute the new Euclidean error:
            euclideanError = math.sqrt((math.pow((goalPoint.x-currentPoint.x),2) + math.pow((goalPoint.y-currentPoint.y),2)))


        # # Recompute Euclidean error if euclideanErro !< threshold:
        # euclideanError = math.sqrt((math.pow((goalPoint.x-currentPoint.x),2) + math.pow((goalPoint.y-currentPoint.y),2)))

        print (" Euclidean Error = ", euclideanError)

        # Case #2:
        if (euclideanError > threshold):

            if reachedGoal:
                # Compute turningRadius , steeringAngle and velocity for current start and goal point:
                senaPurePursuit.compute_turning_radius(currentPoint, goalPoint)

                command = Point32()
                command.x = senaPurePursuit.compute_steering_angle()
                command.y = senaPurePursuit.compute_forward_velocity()

            reachedGoal = False

            # Publish the computed command:
            pub.publish(command)

            # Recompute the Euclidean error to see if its reducing:
            euclideanError = math.sqrt((math.pow((goalPoint.x-currentPoint.x),2) + math.pow((goalPoint.y-currentPoint.y),2)))



        rate.sleep()

    rospy.spin()

if __name__ == '__main__':
command()
