#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerRequest
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

import hello_helpers.hello_misc as hm
import math

def rowtracker(lidar_scan):

    print(lidar_scan.ranges)


if __name__ == '__main__':

        try:
            while not rospy.is_shutdown():

                lidar_subscriber = rospy.init_node('navigation_main', anonymous=True)
                rospy.Subscriber("scan", LaserScan, rowtracker)
                rospy.spin()

            

        except KeyboardInterrupt:
            rospy.loginfo('interrupt received bro!!! Gotcha!')