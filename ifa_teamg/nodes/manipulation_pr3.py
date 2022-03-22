#!/usr/bin/env python

from cv2 import transform
import rospy
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint

import tf2_ros
import tf2_geometry_msgs
from tf2_geometry_msgs import PoseStamped 

import hello_helpers.hello_misc as hm

class Manipulation_pr3(hm.HelloNode):
   
	def __init__(self):
		hm.HelloNode.__init__(self)
		self.pose = None

	#def turnOnManipulationMode(self):
	#    pass

	# def move_to_initial_configuration(self):
	# 	initial_pose = {'wrist_extension': 0.01,
	# 					'joint_lift': 0.5,}

	# 	rospy.loginfo('Move to the initial position.')
	# 	self.move_to_pose(initial_pose)


	def moveWristToMarker(self, pose):
		
		# transform 'aruco_frame:base_link' to 'base_link'-> might want to try link_lift? 
		target_frame = 'base_link' 
		print(pose.header.frame_id[1:])

		transform = self.tf_buffer.lookup_transform(target_frame, pose.header.frame_id[1:], pose.header.stamp)
		pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, transform)

		# returns trans:xyz orien:xyzw
		offset_y = -0.230 #changing this to positive may be a way to calibrate the offset. 
		offset_z = 0.122#0.160#0.210 #0.185 #0.250 #robot's arm is lower than the center of aruco marker.. why?
		print(pose_transformed)
		pose_z = pose_transformed.pose.position.z - offset_z
		pose_y = -(pose_transformed.pose.position.y - offset_y)

		pose_desired = {'joint_lift' : pose_z, 'wrist_extension' : pose_y }
		print(pose_desired)
		self.move_to_pose(pose_desired)
		
	# def moveToolToMarker(poses):
	#   pass

	def main(self):
		#rospy.init_node("manipulation_pr3", anonymous=True)
		hm.HelloNode.main(self, 'manipuation_pr3', 'manipulation_pr3', wait_for_first_pointcloud=False)
		self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10))
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
		self.wrist_sub = rospy.Subscriber("/aruco_simple/pose", PoseStamped, self.moveWristToMarker, queue_size=1)

if __name__ == '__main__':
	try:
		node = Manipulation_pr3()      
		node.main()       
		# wrist_sub = rospy.Subscriber("/aruco_single/pose", PoseStamped, node.moveWristToMarker)
		rospy.spin()

	except KeyboardInterrupt:
		rospy.loginfo('Interrupt received, so shutting down')
