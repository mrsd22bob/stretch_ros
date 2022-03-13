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
	self.tf_buffer = None
        self.tf_listener = None

    def moveWristToMarker(self, pose):
        
        target_frame = 'link_arm_l4'
	print(pose.header.frame_id[1:])
	
        transform = self.tf_buffer.lookup_transform(target_frame, pose.header.frame_id[1:], pose.header.stamp)

        rospy.Duration(1.0)

        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose, transform)
        #returns trans:xyz orien:xyzw

        pose_desired = {'wrist_extension' : pose_transformed.pose.position.y, 'joint_lift' : pose_transformed.pose.position.z}
	print(pose_desired)
        self.move_to_pose(pose_desired)


    # def moveToolToMarker(poses):
    #   pass

    def main(self):
        hm.HelloNode.main(self, 'manipuation_pr3', 'manipulation_pr3', wait_for_first_pointcloud=False)

	self.tf_buffer = tf2_ros.Buffer(rospy.Duration(10))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        

if __name__ == '__main__':
    try:
        node = Manipulation_pr3()       
        node.main()
	#rospy.init_node("manipulation_pr3", anonymous=True)
        wrist_sub = rospy.Subscriber("/aruco_single/pose", PoseStamped, node.moveWristToMarker)
        rospy.spin()

    except KeyboardInterrupt:
        rospy.loginfo('Interrupt received!')


	

