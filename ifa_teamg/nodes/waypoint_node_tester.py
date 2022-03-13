#!/usr/bin/env python
 
import time

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry

rospy.init_node('way_point_tester', anonymous=True)


wyptPub = rospy.Publisher('way_point', PoseStamped, queue_size=10)
odomPub = rospy.Publisher('odom', Odometry, queue_size=10)

#Waypoint test message
wyptMessage = PoseStamped()
wyptMessage.header.seq = 0
wyptMessage.header.stamp = rospy.Time.now()
wyptMessage.header.frame_id = "odom"
wyptMessage.pose.position.x = 0
wyptMessage.pose.position.y = 0
wyptMessage.pose.position.z = 0
wyptMessage.pose.orientation.x = 0
wyptMessage.pose.orientation.y = 0
wyptMessage.pose.orientation.z = 0
wyptMessage.pose.orientation.w = 0


#Odom test message
odomMessage = Odometry()
odomMessage.header.seq = 0
odomMessage.header.stamp = rospy.Time.now()
odomMessage.header.frame_id = "odom"
odomMessage.child_frame_id = "base_link"
odomMessage.pose.pose.position.x = 0
odomMessage.pose.pose.position.y = 0
odomMessage.pose.pose.position.z = 0
odomMessage.pose.pose.orientation.x = 0
odomMessage.pose.pose.orientation.y = 0
odomMessage.pose.pose.orientation.z = 0
odomMessage.pose.pose.orientation.w = 0





# Uncomment to test wayypoint list update
"""
while True:

    wyptMessage.header.seq += 1
    wyptMessage.header.stamp = rospy.Time.now()
    wyptMessage.header.frame_id = "odom"

    wyptMessage.pose.position.x += 1
    wyptMessage.pose.position.y += 1
    wyptMessage.pose.position.z += 0

    wyptMessage.pose.orientation.x -= 0
    wyptMessage.pose.orientation.y += 0
    wyptMessage.pose.orientation.z -= 0
    wyptMessage.pose.orientation.w = 1

    wyptPub.publish(wyptMessage)

    time.sleep(10)"""

def publishit(odomPub, odomMessage):

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        odomPub.publish(odomMessage)
        rate.sleep()

        odomMessage.header.seq += 1
        odomMessage.header.stamp = rospy.Time.now()
        odomMessage.header.frame_id = "odom"
        odomMessage.child_frame_id = "base_link"
        odomMessage.pose.pose.position.x = 0
        odomMessage.pose.pose.position.y += 50
        odomMessage.pose.pose.position.z -= 25
        odomMessage.pose.pose.orientation.x -= 100
        odomMessage.pose.pose.orientation.y = 0
        odomMessage.pose.pose.orientation.z = 0
        odomMessage.pose.pose.orientation.w += 1

        # odomPub.publish(odomMessage)

        # print("abc")

        time.sleep(5)


if __name__ == '__main__':
    try:
        # odomPub.publish(odomMessage)
        publishit(odomPub, odomMessage)
    except rospy.ROSInterruptException:
        pass