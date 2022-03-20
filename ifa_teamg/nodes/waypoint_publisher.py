#!/usr/bin/env python

from re import X

from cv2 import line
import rospy
from std_srvs.srv import Trigger, TriggerRequest
from sensor_msgs.msg import JointState, PointCloud2, LaserScan
from control_msgs.msg import FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path

from move_base_msgs.msg import MoveBaseActionGoal

#for quaternion tranformation from Euler to Quaternion
from tf.transformations import quaternion_from_euler

from scipy import optimize

import hello_helpers.hello_misc as hm
import math
import ros_numpy as rnp
# from ros_numpy import point_cloud2
import numpy as np

import laser_geometry.laser_geometry as lg
import matplotlib.pyplot as plt

lp = lg.LaserProjection()


class Robot(hm.HelloNode):

    def __init__(self):
        hm.HelloNode.__init__(self)

        self.rate = 10.0

        self.square_side = 3.5
        self.rotation_angle = -math.pi/2  # In radians
        self.mode = 'position'
        self.straight_command = {'joint': 'translate_mobile_base', 'inc': self.square_side}
        self.cw_command = {'joint': 'rotate_mobile_base', 'inc': -self.rotation_angle}
        self.ccw_command = {'joint': 'rotate_mobile_base', 'inc': self.rotation_angle}

        self.pc_baselink = None

        self.cluster_threshold = 0.02

        self.centroids = []

        self.cluster_max_points = 30

        self.pot_radius_max_threshold = 0.20

        self.pot_radius_min_threshold = 0.05

        self.pot_spacing = 0.5

        self.waypoints = []

        self.safe_dist = 0.35

        


        #Using the twist for rudimentary motion implementation without using move_base
        self.twist_msg = Twist()

        #Using MoveBaseActionGoal for motion implementation using move_base
        # self.move_msg = MoveBaseActionGoal()
        # self.move_msg_count = 0

        self.move_msg_simple = PoseStamped()
        self.move_msg_count = 0

        #Velocity Publisher for the base
        self.velocity_pub = rospy.Publisher("/stretch/cmd_vel", Twist, queue_size=1)

        #Move-base Publisher for the base
        # self.move_base_goal_pub = rospy.Publisher("move_base/goal", MoveBaseActionGoal, queue_size=1)

        #Move-base Simple Goal Publisher for the base
        self.move_base_simple_goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)
        
        #Publisher for the entire point cloud represented in base_link frame
        # self.baselink_pc_pub = rospy.Publisher("/pc_in_laser", PointCloud2, queue_size=1)

        #Publisher for the point cloud represented in base_link frame cropped to the relevant view window to the right of the robot
        # self.baselink_pc_pub_vw = rospy.Publisher("/laser_in_base", PointCloud2, queue_size=1)

    #def display_test_stats():

    def straight_motion(self, goal_point, trajectory_goal):

        joint_name = self.straight_command['joint']
        trajectory_goal.trajectory.joint_names = [joint_name]

        goal_point.positions = [self.straight_command['inc']]
        trajectory_goal.trajectory.points = [goal_point]
        trajectory_goal.trajectory.header.stamp = rospy.Time.now()
        self.trajectory_client.send_goal(trajectory_goal)

        self.trajectory_client.wait_for_result()

    def process_cloud(self, pc_baselink_box):

        print("Processing Cloud")
        
        # print(pc_baselink_box.header.frame_id)
        xyz = rnp.point_cloud2.pointcloud2_to_xyz_array(pc_baselink_box).T

        # print(xyz.shape)

        index_to_keep_x = np.where((xyz[0,:] > 0))
        xyz_selfx = xyz[:,index_to_keep_x].reshape(3,-1)

        # print(xyz_selfx.shape)
        self.find_centers(xyz_selfx)

    def odom_update(self, odom_feedback):

        self.x_current = odom_feedback.pose.pose.position.x
        self.y_current = odom_feedback.pose.pose.position.y

    
    def find_centers(self, xyz):

        # print(xyz.shape)

        # print(xyz)

        if xyz.shape[1] == 0:
            return 0

        cluster = np.asarray(xyz[:,0])
        cluster = cluster.reshape((-1,1))

        for i in range(1,xyz.shape[1]):   
   

            if np.linalg.norm([xyz[0,i] - xyz[0,i-1], xyz[1,i] - xyz[1,i-1]]) < self.cluster_threshold and cluster.shape[1] < self.cluster_max_points:
                # print(cluster.shape)
                # print(xyz[:,i].reshape((-1,1)).shape)
                cluster = np.hstack((cluster, xyz[:,i].reshape((-1,1))))
                

                # print(np.linalg.norm([xyz[0,i] - xyz[0,i-1], xyz[1,i] - xyz[1,i-1]]))
                # print(cluster)
            
            else:
                #Check if number of points in already collected cluster are > 3, if not continue 
                if cluster.shape[1] < 3:
                    continue

                #If there are > 3 points, compute centroid and add to centroid collection
                elif cluster.shape[1] <= self.cluster_max_points:

                    #Best fit arc and corresponding cetroid
                    [cent_x, cent_y, R_xy] = find_centroids(cluster)

                    # print(R_xy)

                    if R_xy < self.pot_radius_max_threshold and R_xy > self.pot_radius_min_threshold:
                        # print("Centroid received is: ", [cent_x, cent_y])

                        if len(self.centroids) == 0:
                            print("Adding Cluster")

                            self.centroids.append([cent_x, cent_y, R_xy])

                            #Adding waypoints in the odometry frame
                            # self.waypoints = np.asarray([[cent_x], [cent_y+self.safe_dist]])

                            #create goal tuple to send to move_base_call
                            transform_base_to_odom = hm.get_p1_to_p2_matrix("base_link", "odom", self.tf2_buffer)[0]
                            goal_vec_odom = np.asarray([[self.waypoints[0,0]], [self.waypoints[1,0] + self.safe_dist], [0], [1]])
                            goal_loc =  np.matmul(transform_base_to_odom, goal_vec_odom)
                            goal_orientation = np.arctan2(goal_vec_odom[1,0] - self.y_current, self.waypoints[0,0] - self.x_current)

                            goal_orientation = quaternion_from_euler(0,0,goal_orientation)

                            pose = PoseStamped()

                            pose.header.frame_id = "odom"
                            pose.header.stamp = rospy.Time.now()
                            pose.pose.position.x = goal_loc[0]
                            pose.pose.position.x = goal_loc[1]

                            pose.pose.orientation = goal_orientation
                            
                            self.nav_path.poses.append(pose)



                        elif np.linalg.norm([self.centroids[-1][0] - cent_x, self.centroids[-1][1] - cent_y]) > self.pot_spacing:
                            print("Adding Cluster")
                            self.centroids.append([cent_x, cent_y, R_xy])

                            #Adding waypoints in the odometry frame
                            # self.waypoints = np.hstack((self.waypoints, np.asarray([[cent_x],[cent_y]])))

                            #create goal tuple to send to move_base_call
                            transform_base_to_odom = hm.get_p1_to_p2_matrix("base_link", "odom", self.tf2_buffer)[0]
                            goal_orientation = np.arctan2(self.waypoints[1,-1] - self.waypoints[1,-2], self.waypoints[0,-1] - self.waypoints[0,-2])
                            goal_orientation = quaternion_from_euler(0,0,goal_orientation)


                            goal_vec_odom = np.asarray([[self.waypoints[0,0] + self.safe_dist*np.cos(goal_orientation + np.pi/2)], [self.waypoints[1,0] + self.safe_dist*np.sin(goal_orientation + np.pi/2)], [0], [1]])
                            goal_loc =  np.matmul(transform_base_to_odom, goal_vec_odom)

                            move_tuple = (goal_loc[0], goal_loc[1], goal_orientation)

                            pose = PoseStamped()

                            pose.header.frame_id = "odom"
                            pose.header.stamp = rospy.Time.now()
                            pose.pose.position.x = goal_loc[0]
                            pose.pose.position.x = goal_loc[1]

                            pose.pose.orientation = goal_orientation
                            
                            self.nav_path.poses.append(pose)
                            

                        print(self.waypoints)

                        # print("Waypoints added succesfully")
                    
                #Restart cluster with current point
                cluster = np.asarray(xyz[:,i])
                cluster = cluster.reshape((-1,1))

                

        if self.centroids == []:
            print("No centroids found!")

        
    def trajectory_tracker(self):

        if self.centroids == []:
            rospy.loginfo("No waypoints detected yet.")

            print("No waypoints detected yet.")


    def main(self):
        hm.HelloNode.main(self,'waypoint_publisher', 'navigation', wait_for_first_pointcloud=False)

        # Initializing Message for Navigation Path
        self.nav_path = Path()
        self.nav_path.header.frame_id = "odom"
        self.nav_path.header.stamp = rospy.Time.now()

def find_centroids(cluster):

    x = cluster[0,:]
    y = cluster[1,:]

    x_m = np.average(x)
    y_m = np.average(y)

    center_estimate = x_m, y_m
    center_2, ier = optimize.leastsq(f_2, center_estimate, args=(cluster))

    xc_2, yc_2 = center_2
    Ri_2       = calc_R(*center_2, cluster=cluster)
    R_2        = Ri_2.mean()
    residu_2   = sum((Ri_2 - R_2)**2)

    return xc_2, yc_2, R_2


def calc_R(xc, yc, cluster):
    """ calculate the distance of each 2D points from the center (xc, yc) """
    x = cluster[0,:]
    y = cluster[1,:]
    return np.sqrt((x-xc)**2 + (y-yc)**2)


def f_2(c, cluster):
    """ calculate the algebraic distance between the data points and the mean circle centered at c=(xc, yc) """
    Ri = calc_R(*c, cluster = cluster)
    # print(Ri - Ri.mean())
    return Ri - Ri.mean()


if __name__ == '__main__':

    try:
        node = Robot()
        node.main()

        lidar_subscriber = rospy.Subscriber('/laser_cloud_baselink', PointCloud2, node.process_cloud)

        odom_subscriber = rospy.Subscriber('/odom', Odometry, node.odom_update)

        rospy.spin()

    except KeyboardInterrupt:
        rospy.loginfo('interrupt received bro!!! Gotcha!')
