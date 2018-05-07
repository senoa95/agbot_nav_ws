import numpy as np
import math
import utm

### Define constants:
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

# class to define Pure pursuit controller parameters
class PPController:

    # Constructor:
    def __init__(self,inputLeadDistance,inputLength):
        self.leadDistance = inputLeadDistance
        self.length = inputLength #set the length for ppcontroller as the length of maximumSteeringAngle
        self.turningRadius = 1
        self.maximumVelocity = 0.6

        # List of waypoints: From start to end:
        self.wpList = []

        # Current target waypoint index:
        self.currWpIdx = 0

        # List of desired heading values:
        self.tgtHeading = []

        # List of normal vectors to segments joining the waypoints:
        self.segNormVecList = None

        # Number of waypoints:
        self.nPts = 0

        # Tuning gains:
        self.k_theta = 0.2

        self.k_delta = 0.65

        self.k_vel = 0.1

        self.minVelocity = 0.1

    # Initialize the controller from a text file containing the waypoints:
    def initialize(self,fileName):

        wpFile = open(fileName, 'r')

        wPts = wpFile.readlines()

        for wp in wPts:

            wpFile = open(fileName, 'r')

            wPts = wpFile.readlines()

        for wp in wPts:

            spLine = wp.split( ',')

            # curr_y, curr_x, zoneNum , char = utm.from_latlon(float(spLine[0]) , float(spLine[1]))

            self.wpList.append(Point(float(spLine[0]) , float(spLine[1])))

        self.nPts = len(self.wpList)
        self.segNormVecList = np.zeros((2,self.nPts))

        self.tgtHeading.append(0)

        #
        #
        #     spLine = wp.split( ',')
        #
        #     self.wpList.append(Point(float(spLine[0]) , float(spLine[1])))
        #
        # self.nPts = len(self.wpList)
        # self.segNormVecList = np.zeros((2,self.nPts))
        #
        # self.tgtHeading.append(0)


        # Loop to compute the target heading values:
        for idx in range(0, len(self.wpList)-1):

            self.tgtHeading.append( math.atan2( self.wpList[idx + 1].y - self.wpList[idx].y , self.wpList[idx+1].x - self.wpList[idx].x))

            normX = self.wpList[idx].y - self.wpList[idx + 1].y
            normY = self.wpList[idx + 1].x - self.wpList[idx].x

            # Calculate the norm:
            nVecMag = np.sqrt( normX**2 + normY**2)

            self.segNormVecList[0,idx+1] = normX/nVecMag
            self.segNormVecList[1,idx+1] = normY/nVecMag

        self.tgtHeading[0] = self.tgtHeading[1]
        self.segNormVecList[:,0] = self.segNormVecList[:,1]



    # Function to compute steering angle and forward velocity commands:
    def compute_steering_vel_cmds(self,current):

        # Compute vector from current position to current waypoint:
        vecRobot2Wp = np.zeros((2,1))
        vecRobot2Wp[0,0] =  self.wpList[self.currWpIdx].x - current.x
        vecRobot2Wp[1,0] =  self.wpList[self.currWpIdx].y - current.y

        # Compute the minimum distance from the current segment:
        minDist = -np.dot(vecRobot2Wp.T, self.segNormVecList[:,self.currWpIdx])


        # Compute the desired heading angle based of target heading and the min dist:
        theta_des = self.tgtHeading[self.currWpIdx]  - self.k_theta*minDist


        # Compute the steering angle command:
        delta = self.k_delta*(theta_des - current.heading)

        # Debugging section:
        # if (self.currWpIdx ==0):
        #     # print ( "Normal Vec = ", self.segNormVecList[:,self.currWpIdx])
        #     # print (" Robot2Wp Vec = ", vecRobot2Wp)
        #     print (" Minimum Dist = ", minDist)
        #     print (" Target heading = ", self.tgtHeading[self.currWpIdx] )
        #     print (" Current heading = " , current.heading)
        #     # print (" Desired Theta = ", theta_des)
        #     print (" Steering angle = ", delta)

        # Compute forward velocity:
        vel = self.maximumVelocity - abs(self.k_vel*delta)

        if vel < self.minVelocity:
            vel = self.minVelocity

        if delta > 1:
            delta = 1

        if delta < -1:
            delta = -1

        return vel,delta

    # compute the steering radius of ackerman vehicle of given parameters
    def compute_turning_radius(self, current = Point(0,0,0) , goal = Point(0,0,0)):

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
