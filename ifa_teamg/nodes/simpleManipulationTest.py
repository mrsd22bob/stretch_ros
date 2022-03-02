#!/usr/bin/env python
import rospy
from std_srvs.srv import Trigger, TriggerRequest
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped

import math
import time
import tf2_ros
import argparse as ap

import hello_helpers.hello_misc as hm
import stretch_body.arm 
import stretch_body.lift

class ManipulationPr2(hm.HelloNode):

    def __init__(self):
        hm.HelloNode.__init__(self)
        # self.desiredPt = {'wrist_extension': 1.0, 'joint_lift': 1.0}
        self.desiredPt_y = 0.0
        self.desiredPt_z = 0.0
        self.min_armlen = 0.2

    def moveArm(self):
        print("Arm start")

        #initialize arm
        a = stretch_body.arm.Arm()
        a.motor.disable_sync_mode()
        if not a.startup():
            exit() # failed to start arm!
            print("Exiting")
        a.home()
        print("Arm homing")
        
        # starting_position = a.status['pos']
        goal_position = self.desiredPt_y
        # move out by 10cm
        a.move_to(goal_position - self.min_armlen)
        a.push_command()
        a.motor.wait_until_at_setpoint()
        a.stop()
        print("Arm done")


    def moveLift(self):
        print("Lift Start")

        #initialize lift
        l = stretch_body.lift.Lift()
        l.motor.disable_sync_mode()
        l.startup()
        if not l.startup():
            print("Exiting")
            exit() # failed to start lift!
        l.home()
        
        starting_position = l.status['pos']
        # goal_position = self.desiredPt_z
        # move up by 10cm
        l.move_to(starting_position)
        # l.move_to(goal_position)
        l.push_command()
        l.motor.wait_until_at_setpoint()
        l.stop()

        l.move_to(starting_position+0.1)
        # l.move_to(goal_position)
        l.push_command()
        l.motor.wait_until_at_setpoint()
        l.stop()

        print("Lift Done")

    def simpleTest(self):
    
        self.desiredPt_y = 0.2
        self.desiredPt_z = 0.2

        self.moveArm()
        self.moveLift()
        

        # rate = rospy.Rate(self.rate)
        # self.move_to_pose(self.desiredPt)

        self.trajectory_client.wait_for_result()

    # def camcoordTest(self):

    def main(self):
        
        hm.HelloNode.main(self,'simpleManipulationTest', 'simpleManipulationTest', wait_for_first_pointcloud=False)
        # subscriber - to ArUco marker coordinates -> which node publishes the coordinates? ;; respondToMarker?
        # self.camcoord_subscriber = rospy.Subscriber("/aruco_single/pose", PoseStamped, respondToMarker)
              
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            
            self.simpleTest()

            rate.sleep()

if __name__ == '__main__':
    try:
        
        parser = ap.ArgumentParser(description='Move lift and arm to the desired position')
        args, unknown = parser.parse_known_args()
        node = ManipulationPr2()
        node.main()

    except KeyboardInterrupt:
        rospy.loginfo('interrupt received, so shutting Bob down')
