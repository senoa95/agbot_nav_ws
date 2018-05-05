#!/usr/bin/env python

# Import libraries:
import rospy
from geometry_msgs.msg import Point32,Pose,Point
from sensor_msgs.msg import NavSatFix

global x

def pose_callback(pose_data):
    rospy.loginfo(rospy.get_caller_id(), pose_data.pose_data)
    x = pose_data.x
    y = pose_data.y
    x = "hahaha"
    print(x)


def attitude_callback(attitude_data):
    rospy.loginfo(rospy.get_caller_id(), attitude_data.attitude_data)
    yaw = attitude_data.z

def waypoint_maker():
    rospy.init_node('waypoint_maker',anonymous=True)
    rospy.Subscriber("/agBOT/Local/Pose", Pose, pose_callback)
    rospy.Subscriber("/imu", Pose, attitude_callback)





    rospy.spin()

if __name__ == '__main__':
    waypoint_maker()
    # attitude_callback()
