#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseStamped, Pose
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf



import time



class NavPlan():

    def __init__(self):

        self.waypoints = []

        #Initializing Subscriber to waypoints
        self.wayptSub = rospy.Subscriber("way_point", PoseStamped, self.AddWaypoints)

        #Initializing Subscriber to waypoints
        self.odomSub = rospy.Subscriber('/odom', Odometry, self.UpdateCurrentPose)

        #Initializing Publisher to robot's cmd_vel topic
        self.twistPub = rospy.Publisher('stretch/cmd_vel', Twist, queue_size=10)

        #Twist message initialization
        self.twistMessage = Twist()

        #Waypoint Tracking Mode
        self.mode = "track"

        #Current Position in Odometry
        self.currentPose = PoseStamped()

        #Initializing Goal Locations
        self.atGoal = Pose()

        self.atGoal.position.x = 0.1
        self.atGoal.position.y = 0
        self.atGoal.position.z = 0

        self.atGoal.orientation.x = 0
        self.atGoal.orientation.y = 0
        self.atGoal.orientation.z = 0
        self.atGoal.orientation.w = 1

        #FOR UNIT TESTING!!!!!! SETTING CURRENT POSE TO ATGOAL TO CHECK TRANSFORMATION
        self.currentPose.pose = self.atGoal

        #Initializing Goal Locations
        self.nextGoal = Pose()

        self.nextGoal.position.x = 1
        self.nextGoal.position.y = 0
        self.nextGoal.position.z = 0

        self.nextGoal.orientation.x = 0
        self.nextGoal.orientation.y = 0
        self.nextGoal.orientation.z = 0.7071
        self.nextGoal.orientation.w = 0.7071

        
        """The parameters that define the cubic path are defined here.  The cubic path parameters
        have to be updated depending on which waypoints are being tracked.  For a given pair of waypoints,
        a cubic trajectory is fit.  When the goal way point is reached, the trajectory is then updated
        to go from the current way point (the earlier goal way point) to the next way_point if there is any."""

        #Cubic Coefficients for x
        self.x_cubic_params = np.asarray([0,0,1,1])
 
        #Cubic Coefficients for y
        self.y_cubic_params = np.asarray([0,0,1,1])

        #Cubic Coefficients for theta
        self.theta_cubic_params = np.asarray([0,0,1,1])

        #Cubic s-parameter Path Coefficient Matrix
        self.s_coeff_mat = np.asarray([[1,1,1,1],[0,0,0,1],[3,2,1,0],[0,0,1,0]])

        #Cubic s-parameter Path Coefficient Matrix Inverse
        self.s_coeff_mat_inv = np.linalg.inv(self.s_coeff_mat)

        

        #Goal Time
        self.path_goal_time = 5

        #Maximum Safe Speed
        self.safe_speed = 0.1



    def StartNode(self):
        #Initializing Waypoint Tracking Node
        rospy.init_node('way_point_tracker', anonymous=True)

        #Path Start Time Offset
        self.path_start_time_offset = rospy.Time.now()

    def UpdateCurrentPose(self, odom_message):

        self.currentPose.header = odom_message.header

        self.currentPose.pose = odom_message.pose.pose

        # print(self.currentPose)

    def TestPub(self):

        rate = rospy.Rate(10)

        while not rospy.is_shutdown():

            self.twistMessage.linear.x += 0.1
            self.twistMessage.linear.y = 0
            self.twistMessage.linear.z = 0

            self.twistMessage.angular.x = 0
            self.twistMessage.angular.y -= 0.4
            self.twistMessage.angular.z = 0

            
            self.twistPub.publish(self.twistMessage)

            rate.sleep()

    #zeroTwist message publication when no motion is to be commanded
    def ZeroTwist(self):

        while True:

            self.twistMessage.linear.x = 0
            self.twistMessage.linear.y = 0
            self.twistMessage.linear.z = 0

            self.twistMessage.angular.x = 0
            self.twistMessage.angular.y = 0
            self.twistMessage.angular.z = 0

            
            self.twistPub.publish(self.twistMessage)

    

    #This is called when after a goal waypoint has been reached
    def UpdatePathParams(self):

        x_constraints = np.asarray([self.nextGoal.position.x, self.atGoal.position.x, 0, 0])
        self.x_cubic_params = np.matmul(self.s_coeff_mat_inv, x_constraints)

        y_constraints = np.asarray([self.nextGoal.position.y, self.atGoal.position.y, 0, 0])
        self.y_cubic_params = np.matmul(self.s_coeff_mat_inv, y_constraints)

        #Converting current and next waypoint orientations into euler angles

        _,_,theta_at = euler_from_quaternion([self.atGoal.orientation.x, self.atGoal.orientation.y, self.atGoal.orientation.z, self.atGoal.orientation.w])
        _,_,theta_next = euler_from_quaternion([self.nextGoal.orientation.x, self.nextGoal.orientation.y, self.nextGoal.orientation.z, self.nextGoal.orientation.w])


        theta_constraints = np.asarray([theta_next, theta_at, 0, 0])
        self.theta_cubic_params = np.matmul(self.s_coeff_mat_inv, theta_constraints)
        
        time.sleep(1)

        a = rospy.Time.now()

        # print(type(a))

        # print(a.secs)
        # print(a.nsecs)

        pass


    def GenerateTwists(self):

        # PD on error + FF term based on time scaling of path
        now = rospy.Time.now()
        s = now.secs + (now.nsecs*1e-9)
        currentPose = self.currentPose.pose

        self.GetTwistError(s, currentPose)

        #Calculating the Feed-forward Term
        x_dot  = 3*self.x_cubic_params[0]*s**2 + 2*self.x_cubic_params[1]*s + self.x_cubic_params[2]
        y_dot = 3*self.y_cubic_params[0]*s**2 + 2*self.y_cubic_params[1]*s + self.y_cubic_params[2]
        theta_dot = 3*self.theta_cubic_params[0]*s**2 + 2*self.theta_cubic_params[1]*s + self.theta_cubic_params[2]

    def GetTwistError(self, s, currentPose):

        H_current = tf.transformations.quaternion_matrix([currentPose.orientation.x,currentPose.orientation.y,currentPose.orientation.z,currentPose.orientation.w])

        print(H_current)
        # current_pos = np.empty((3,1))
        H_current[:,3] = np.array([currentPose.position.x, currentPose.position.y, currentPose.position.z, 1])

        theta_traj = self.theta_cubic_params[0]*s**3 + self.theta_cubic_params[1]*s**2 + self.theta_cubic_params[2]*s + self.theta_cubic_params[3]
        x_traj = self.x_cubic_params[0]*s**3 + self.x_cubic_params[1]*s**2 + self.x_cubic_params[2]*s + self.x_cubic_params[3]
        y_traj = self.y_cubic_params[0]*s**3 + self.y_cubic_params[1]*s**2 + self.y_cubic_params[2]*s + self.y_cubic_params[3]

        H_traj = tf.transformations.euler_matrix(0,0,theta_traj)
        H_traj[:,3] = np.array([x_traj, y_traj, 0, 1])


    
    def TrackWaypoints(self):

        #Check the error between current pose and goal pose
            #Command twists as long as error threshold is violated

        #If error threshold is met
            #set self.atGoal to the goal waypoint that was being trackced
            #set self.nextGoal to the next waypoint if it exists
            #update trjectory coefficients using UpdatePathParams
            #reset path timings as well

        pass
    
    def AddWaypoints(self, waypoint):

        self.waypoints.append(waypoint)




navPlan = NavPlan()
navPlan.StartNode()
# navPlan.UpdatePathParams()
navPlan.GenerateTwists()
# navPlan.testPub()
rospy.spin()