#!/usr/bin/env python

from cv2 import transform
import rospy
import numpy as np
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
		
		
		self.avg_boundingbox_position = None

		self.offset_y = None
		self.offset_z = None
		self.pose_lift = None
		self.pose_arm = None

		self.init_pose = {'joint_lift' : 0.5, 'wrist_extension' : 0.0}
		self.retracted_pose = {'wrist_extension' : 0.0}
		# self.pose_arm_retract = {'wrist_extension' : 0.0}

		self.target_frame = None

		self.arrivedBoundingBox = False
		self.arrivedMarker = False
	#def turnOnManipulationMode(self):
	#    pass

	# def move_to_initial_configuration(self):
	# 	rospy.loginfo('Move to the initial position.')
	# 	self.move_to_pose(self.init_pose)

	# # then, the robot needs to stop. 

	def moveWristToBoundingBox(self, pose):

		if self.arrivedBoundingBox == True:
			return 
		# rospy.loginfo('Move to the initial position.')
		# self.move_to_pose(self.init_pose)
		rospy.loginfo('Move to the arm retracted position.')
		self.move_to_pose(self.retracted_pose)
		self.trajectory_client.wait_for_result()			
		
		# transform 'aruco_frame:base_link' to 'base_link'-> might want to try link_lift? no
		self.target_frame = 'base_link' 
		print(pose.header.frame_id[1:])

		transform = self.tf_buffer.lookup_transform(self.target_frame, pose.header.frame_id[1:], pose.header.stamp)
		pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, transform)
		print(pose_transformed)

		# print(type(pose_transformed.pose.position.z))

		temp_z = pose_transformed.pose.position.z
		temp_y = pose_transformed.pose.position.y		
		received_position = np.empty((20,2))

		for i in range(20):
			for j in range(2):
				if j % 2 ==0:
					received_position[i,j] = temp_z
				else:
					received_position[i,j] = temp_y
			
		self.avg_boundingbox_position = np.average(received_position, axis=0)
		self.avg_boundingbox_position = self.avg_boundingbox_position.tolist()

		# print(type(self.avg_marker_position[0]))
		self.offset_y = -0.150 # should be as far as where enough FOV of wrist camera can be achieved
		self.offset_z = -0.250 #-0.122#0.160#0.210 #0.185  #robot's arm is lower than the center of aruco marker.. why?	
		
		# self.move_to_pose({'joint_lift' :pose_transformed.pose.position.z + self.offset_z})

		
		self.pose_lift = (self.avg_boundingbox_position[0] + self.offset_z)
		self.pose_arm = -(self.avg_boundingbox_position[1] - self.offset_y)
		self.pose_desired = {'joint_lift' : self.pose_lift, 'wrist_extension' : self.pose_arm}
		self.pose_desired_lift = {'joint_lift' : self.pose_lift}
		self.pose_desired_arm = {'wrist_extension' : self.pose_arm}


		self.move_to_pose(self.pose_desired_lift)
		self.trajectory_client.wait_for_result()	
		self.move_to_pose(self.pose_desired_arm)
		self.trajectory_client.wait_for_result()

		self.arrivedBoundingBox = True
		#publish "goal complete"
			
	def moveWristToMarker(self, pose):
		if self.arrivedMarker == True:
			return 
	
		
		# transform 'aruco_frame:base_link' to 'base_link'-> might want to try link_lift? no
		self.target_frame = 'base_link' 
		print(pose.header.frame_id[1:])

		transform = self.tf_buffer.lookup_transform(self.target_frame, pose.header.frame_id[1:], pose.header.stamp)
		pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, transform)
		print(pose_transformed)

		# print(type(pose_transformed.pose.position.z))

		temp_z = pose_transformed.pose.position.z
		temp_y = pose_transformed.pose.position.y		
		received_position = np.empty((20,2))

		for i in range(20):
			for j in range(2):
				if j % 2 ==0:
					received_position[i,j] = temp_z
				else:
					received_position[i,j] = temp_y
			
		avg_marker_position = np.average(received_position, axis=0)
		avg_marker_position = avg_marker_position.tolist()

		# print(type(self.avg_marker_position[0]))
		self.offset_y = -0.150 # should be as far as where enough FOV of wrist camera can be achieved
		self.offset_z = -0.250 #-0.122#0.160#0.210 #0.185  #robot's arm is lower than the center of aruco marker.. why?	
		
		# self.move_to_pose({'joint_lift' :pose_transformed.pose.position.z + self.offset_z})

		
		self.pose_lift = (avg_marker_position[0] + self.offset_z)
		self.pose_arm = -(avg_marker_position[1] - self.offset_y)
		self.pose_desired = {'joint_lift' : self.pose_lift, 'wrist_extension' : self.pose_arm}
		self.pose_desired_lift = {'joint_lift' : self.pose_lift}
		self.pose_desired_arm = {'wrist_extension' : self.pose_arm}


		self.move_to_pose(self.pose_desired_lift)
		self.trajectory_client.wait_for_result()	
		self.move_to_pose(self.pose_desired_arm)
		self.trajectory_client.wait_for_result()

		self.arrivedMarker = True
		#publish "goal complete"

	def main(self):
		#rospy.init_node("manipulation_pr3", anonymous=True)
		hm.HelloNode.main(self, 'manipulation_pr3', 'manipulation_pr3', wait_for_first_pointcloud=False)
		self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10))
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
		
		# self.wrist_sub = rospy.Subscriber("/aruco_simple/pose", PoseStamped, self.moveWristToBoundingBox, queue_size=1)
		self.wrist_sub = rospy.Subscriber("/aruco_simple/pose", PoseStamped, self.moveWristToMarker, queue_size=1)


if __name__ == '__main__':
	try:
		node = Manipulation_pr3()      
		node.main()       
		# wrist_sub = rospy.Subscriber("/aruco_single/pose", PoseStamped, node.moveWristToMarker)
		rospy.spin()

	except KeyboardInterrupt:
		rospy.loginfo('Interrupt received, so shutting down')
