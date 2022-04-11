#!/usr/bin/env python

from __future__ import print_function
import time
from cv2 import transform
import rospy
import numpy as np
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from harvester import harvest
import tf2_ros
import tf2_geometry_msgs
from tf2_geometry_msgs import PoseStamped 
import vfa_pollinator as pollinator
import vfa_cutter as cutter
import vfa_gripper as gripper


import hello_helpers.hello_misc as hm

from behavioral_planner.srv import *
import rospy

state  = 0
# 0 - head camera
# 1 - wrist camera

man_no_op_key = 13
man_no_op_over_key = 14
man_op_key = 15
man_op_over_key = 16

process_over_key = 56

man_no_op_flag = 0
man_op_flag = 0
process_over = 0

man_no_op_over = 0
man_op_over = 0

def bp_to_man_handle(req):
	if(req.req==man_no_op_key):
		print("MAN: Manipulation without operation begins")
		global man_no_op_flag
		man_no_op_flag = 1
		return bpServiceResponse(req.req)
	if(req.req==man_op_key):
		print("MAN: Manipulation with operation begins")
		global man_op_flag
		man_op_flag = 1
		return bpServiceResponse(req.req)
	if(req.req==process_over_key):
		print("MAN: Process over, exiting")
		global process_over
		process_over = 1
		return bpServiceResponse(req.req)
	else:
		return bpServiceResponse(0)

def bp_to_man_server():
	# rospy.init_node('bp_to_man_server')
	s = rospy.Service('bp_to_man', bpService, bp_to_man_handle)
	print("MAN: Ready")

def nodes_to_bp_client(key):
	rospy.wait_for_service('nodes_to_bp')
	try:
		req = rospy.ServiceProxy('nodes_to_bp', bpService)
		resp = req(key)
		return resp.reply
	except rospy.ServiceException as e:
		print("Service call failed: %s"%e)


class Manipulation_pr4(hm.HelloNode):
   
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

		hm.HelloNode.main(self, 'manipulation_pr4', 'manipulation_pr4', wait_for_first_pointcloud=False)
		self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10))
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
		self.wristbb_sub = rospy.Subscriber("/aruco_head/pose3", PoseStamped, self.moveWristToBoundingBox, queue_size=1) #49


	#def turnOnManipulationMode(self):
	#    pass

	# def move_to_initial_configuration(self):
	#   rospy.loginfo('Move to the initial position.')
	#   self.move_to_pose(self.init_pose)

	# # then, the robot needs to stop. 

	def harvest_home(self):
		c=cutter.VFACutter()
		g=gripper.VFAGripper()
		if not g.startup():
				exit()
		if not c.startup():
				exit()
		print("Homing gripper")
		g.home()
		print("Homing cutter")
		c.home()
		time.sleep(3.0)
		g.open()
		c.open()
		time.sleep(3.0)
		g.stop()
		c.stop()
	
	def harvest(self):
		c=cutter.VFACutter()
		g=gripper.VFAGripper()
		if not g.startup():
				exit()
		if not c.startup():
			exit()
		print("Gripping")
		g.close()
		print("Cutting")
		c.close()
		time.sleep(2.0)
		print("Re-cutting")
		c.open_slight()
		time.sleep(2.0)
		c.close()
		time.sleep(2.0)
		print("Releasing cutter")
		c.open()
		time.sleep(2.0)
		g.stop()
		c.stop()

	def pollinate(self):
		p=pollinator.VFAPollinator()

		if not p.startup():
			exit()	
		print("moving to start position")
		p.movetostartpos()
		time.sleep(0.5)

		#start vibration

		p.movetopos2()
		time.sleep(0.5)
		p.movetopos3()
		time.sleep(0.5)
		p.movetozero()
		#stop vibration
		time.sleep(3.0)
		p.stop()


	def moveWristToBoundingBox(self, pose):

		# if self.arrivedBoundingBox == True:
		#     return 
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

		temp_lift = pose_transformed.pose.position.z
		temp_arm = pose_transformed.pose.position.y       
		received_position = np.empty((20,2))

		for i in range(20):
			for j in range(2):
				if j % 2 ==0:
					received_position[i,j] = temp_lift
				else:
					received_position[i,j] = temp_arm
			
		self.avg_boundingbox_position = np.average(received_position, axis=0)
		self.avg_boundingbox_position = self.avg_boundingbox_position.tolist()

		# print(type(self.avg_marker_position[0]))
		self.offset_y = -0.250 - 0.250 # should be as far as where enough FOV of wrist camera can be achieved
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

		# self.arrivedBoundingBox = True
		# global man_no_op_over
		# man_no_op_over = 1
		self.wristbb_sub.unregister()
		self.wristpoi_sub = rospy.Subscriber("/aruco_wrist/pose1", PoseStamped, self.moveWristToMarker, queue_size=1) #39

		#publish "goal complete"
			
	def moveWristToMarker(self, pose):
		# if self.arrivedMarker == True:
		#     return 
		
		# transform 'aruco_frame:base_link' to 'base_link'-> might want to try link_lift? no
	
		initial_pos= self.pose_arm
		self.target_frame = 'base_link' 
		print(pose.header.frame_id[1:])
		self.retractarm= {'wrist_extension' :0.0}
		self.lowerlift={'joint_lift' :0.5}
	


		transform = self.tf_buffer.lookup_transform(self.target_frame, pose.header.frame_id[1:], pose.header.stamp)
		pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, transform)
		print(pose_transformed)

	   

		temp_lift = pose_transformed.pose.position.z
		temp_arm = pose_transformed.pose.position.y       
		received_position = np.empty((20,2))

		for i in range(20):
			for j in range(2):
				if j % 2 ==0:
					received_position[i,j] = temp_lift
				else:
					received_position[i,j] = temp_arm
			
		avg_marker_position = np.average(received_position, axis=0)
		avg_marker_position = avg_marker_position.tolist()

		# print(type(self.avg_marker_position[0]))
		# self.offset_y = -0.150 # should be as far as where enough FOV of wrist camera can be achieved
		# self.offset_z = -0.250 #-0.122#0.160#0.210 #0.185  #robot's arm is lower than the center of aruco marker.. why? 
		
		# self.move_to_pose({'joint_lift' :pose_transformed.pose.position.z + self.offset_z})

		
		self.pose_lift = (avg_marker_position[0]-.12)
	  
		self.pose_arm = -(avg_marker_position[1])
		print("self pose arm- avg marker position")
		print(self.pose_arm)
		print(self.pose_lift)

		
		self.pose_desired = {'joint_lift' : self.pose_lift, 'wrist_extension' : self.pose_arm-.37}
		self.pose_desired_lift = {'joint_lift' : self.pose_lift}
		self.pose_desired_arm = {'wrist_extension' :self.pose_arm-.37}
		
		self.harvest_home() #add only once in main start code
		self.move_to_pose(self.pose_desired_lift)
		self.trajectory_client.wait_for_result()    
		self.move_to_pose(self.pose_desired_arm)
		self.trajectory_client.wait_for_result()
		self.harvest()
			#self.pollinate()
		self.move_to_pose(self.retractarm)
		self.move_to_pose(self.lowerlift)
		self.trajectory_client.wait_for_result()
		self.g.open()
		self.trajectory_client.wait_for_result()
			

		# self.arrivedMarker = True
		global man_op_over
		man_op_over = 1
		#publish "goal complete"

	# def main(self):
	#     #rospy.init_node("manipulation_pr3", anonymous=True)
	#     hm.HelloNode.main(self, 'manipulation_pr4', 'manipulation_pr4', wait_for_first_pointcloud=False)
	#     self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10))
	#     self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
		
	#     self.wristbb_sub = rospy.Subscriber("/aruco_head/pose3", PoseStamped, self.moveWristToBoundingBox, queue_size=1) #49
	#     self.wristpoi_sub = rospy.Subscriber("/aruco_wrist/pose1", PoseStamped, self.moveWristToMarker, queue_size=1) #39


if __name__ == '__main__':
	try:
		# bp_to_man_server() 
		node = Manipulation_pr4()  
		
		rospy.spin()

		# while(not rospy.is_shutdown()):
		#     if(man_no_op_flag==1):
		#         node.wristbb_sub = rospy.Subscriber("/aruco_head/pose3", PoseStamped, node.moveWristToBoundingBox, queue_size=1)
		#         #man_no_op
		#         while(man_no_op_over==0):
		#             continue
		#         #now man_no_op_over = 1

		#         #reset man_no_op
		#         node.wristbb_sub.unregister()
		#         man_no_op_flag=0
		#         man_no_op_over=0
		#         #send man_no_op done to BP
		#         if(nodes_to_bp_client(man_no_op_over_key)==man_no_op_over_key):
		#             ("MAN: Manipulationprint without operation over")

		#     if(man_op_flag==1):
		#         #man_op
		#         node.wristpoi_sub = rospy.Subscriber("/aruco_wrist/pose1", PoseStamped, node.moveWristToMarker, queue_size=1)
		#         while(man_op_over==0):
		#             continue
		#         #now man_op_over = 1
		#         #reset man_op
		#         node.wristpoi_sub.unregister()
		#         man_op_flag=0
		#         man_op_over=0
		#         #send done to BP
		#         if(nodes_to_bp_client(man_op_over_key)==man_op_over_key):
		#             print("MAN: Manipulation with operation over")

		#     if(process_over == 1):
		#         break


	except KeyboardInterrupt:
		rospy.loginfo('Interrupt received, so shutting down')
