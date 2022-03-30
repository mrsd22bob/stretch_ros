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
		self.reach = False

		self.offset_y = 0
		self.offset_z = 0
		self.pose_arm_retract = {'wrist_extension' : 0.0}
		self.pose_lift = 0
		self.pose_arm = 0

		self.pose_desired = {'joint_lift' : self.pose_lift, 'wrist_extension' : self.pose_arm}
		self.pose_desired_lift = {'joint_lift' : self.pose_lift}
		self.pose_desired_arm = {'wrist_extension' : self.pose_arm}

	#def turnOnManipulationMode(self):
	#    pass

	# def move_to_initial_configuration(self):
	# 	initial_pose = {'wrist_extension': 0.01,
	# 					'joint_lift': 0.5,}

	# 	rospy.loginfo('Move to the initial position.')
	# 	self.move_to_pose(initial_pose)

	# def moveWristTOBoundingBox(self, pose):
	# 	target_frame = 'base_link' 
	# 	print(pose.header.frame_id[1:])

	# 	transform = self.tf_buffer.lookup_transform(target_frame, pose.header.frame_id[1:], pose.header.stamp)
	# 	pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, transform)

	# 	# returns trans:xyz orien:xyzw
	# 	offset_y = -0.230 #changing this to positive may be a way to calibrate the offset. 
	# 	offset_z = 0.122#0.160#0.210 #0.185 #0.250 #robot's arm is lower than the center of aruco marker.. why?
	# 	print(pose_transformed)
	# 	pose_z = pose_transformed.pose.position.z - offset_z
	# 	pose_y = -(pose_transformed.pose.position.y - offset_y)


	# 	pose_arm_retract = {'wrist_extension' : 0.0}
	# 	pose_desired = {'joint_lift' : pose_z, 'wrist_extension' : pose_y }
	# 	print(pose_desired)
	# 	pose_desired_z = {'joint_lift' : pose_z}
	# 	pose_desired_y = {'wrist_extension' : pose_y}
	# 	self.move_to_pose(pose_arm_retract)
	# 	self.move_to_pose(pose_desired_z)
	# 	self.trajectory_client.wait_for_result()
	# 	self.move_to_pose(pose_desired_y)
	# 	self.trajectory_client.wait_for_result()
	# # then, the robot needs to stop. 

	def moveWristToBoundingBox(self, pose):


		self.offset_y = -0.150 #changing this to positive may be a way to calibrate the offset. 
		self.offset_z = -0.250 #-0.122#0.160#0.210 #0.185  #robot's arm is lower than the center of aruco marker.. why?

		self.move_to_pose(self.pose_arm_retract)
		self.trajectory_client.wait_for_result()	
		
		# transform 'aruco_frame:base_link' to 'base_link'-> might want to try link_lift? no
		target_frame = 'base_link' 
		print(pose.header.frame_id[1:])

		transform = self.tf_buffer.lookup_transform(target_frame, pose.header.frame_id[1:], pose.header.stamp)
		pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, transform)
		print(pose_transformed)
		# make threshold for the pose to check if it should be updated as a new pose or not 

		self.pose_lift = (pose_transformed.pose.position.z + self.offset_z)
		self.pose_arm = -(pose_transformed.pose.position.y - self.offset_y)

		
		pose_desired = {'joint_lift' : self.pose_lift, 'wrist_extension' : self.pose_arm}
		pose_desired_z = {'joint_lift' : self.pose_lift}
		pose_desired_y = {'wrist_extension' : self.pose_arm}
		print(pose_desired)
		
		if self.reach != True:	
			# Actuate the arm 

			
			self.move_to_pose(pose_desired_z)
			self.trajectory_client.wait_for_result()
			
			self.move_to_pose(pose_desired_y)
			self.trajectory_client.wait_for_result()
			self.reach = True
			break

			
			#then, the robot needs to stop. 
			# pose_desired = {'joint_lift' : pose_z, 'wrist_extension' : pose_y }
			

	# def moveToolToMarker(poses):
	#   pass

	def main(self):
		#rospy.init_node("manipulation_pr3", anonymous=True)
		hm.HelloNode.main(self, 'manipulation_pr3', 'manipulation_pr3', wait_for_first_pointcloud=False)
		self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10))
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
		# self.wrist_sub = rospy.Subscriber("BoundingBoxTopic", PoseStamped, self.moveWristToMarker, queue_size=1)
		self.wrist_sub = rospy.Subscriber("/aruco_simple/pose", PoseStamped, self.moveWristToBoundingBox, queue_size=1)



if __name__ == '__main__':
	try:
		node = Manipulation_pr3()      
		node.main()       
		# wrist_sub = rospy.Subscriber("/aruco_single/pose", PoseStamped, node.moveWristToMarker)
		rospy.spin()

	except KeyboardInterrupt:
		rospy.loginfo('Interrupt received, so shutting down')
