#!/usr/bin/env python

from __future__ import print_function

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

from sensor_msgs.msg import PointCloud2

from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse

import math
import time
import threading
import sys
import tf2_ros
import argparse as ap        
import numpy as np
import os

import hello_helpers.hello_misc as hm
import stretch_funmap.navigate as nv
import stretch_funmap.manipulation_planning as mp


class pollinate(hm.HelloNode):
     def __init__(self):
        hm.HelloNode.__init__(self)

        self.rate = 10.0
       # self.joint_states = None
        #self.joint_states_lock = threading.Lock()

        #self.move_base = nv.MoveBase(self)
        #self.letter_height_m = 0.2
        #self.letter_top_lift_m = 1.08 #1.05
        
     #def joint_states_callback(self, joint_states):
        #self.joint_states = joint_states

     def move_to_initial_configuration(self):
        initial_pose = {#'wrist_extension': 0.01,
                        'joint_wrist_yaw': 0.0,
                        #'gripper_aperture': 0.125}
        }
        #rospy.loginfo('Moved to the initial configuration for pollination.')
        self.move_to_pose(initial_pose)

     def move_wrist(self):
        #rospy.loginfo('moving wrist')
        pose = {'joint_wrist_yaw': 0.8}
        self.move_to_pose(pose)

    

     def main(self):
        hm.HelloNode.main(self, 'pollinate', 'pollinate', wait_for_first_pointcloud=False)

    
if __name__ == '__main__':
    try:
	while not rospy.is_shutdown():

        #parser = ap.ArgumentParser(description='Grasp Object behavior for stretch.')
       #args, unknown = parser.parse_known_args()
       	 node = pollinate()
       	 node.main()
      	 node.move_to_initial_configuration()
     	 node.move_wrist()

    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting down')
