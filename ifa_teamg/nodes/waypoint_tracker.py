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

        self.atGoal.position.x = 0
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

        #Control Gains

        self.Kpx = 0.1
        self.Kpy = 0.1
        self.Kpz = 0.1

        self.Kdx = 0.1
        self.Kdy = 0.1
        self.Kdz = 0.1

        
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

        #path duration
        self.path_duration = 5

        #Goal Time
        self.path_goal_time = 5

        #Maximum Safe Speed
        self.safe_speed = 0.1



    def StartNode(self):
        #Initializing Waypoint Tracking Node
        rospy.init_node('way_point_tracker', anonymous=True)

        #Path Start Time Offset
        self.path_start_time_offset = rospy.Time.now()

        # Creating Transform Listener
        self.tf_listener = tf.TransformListener()

        print("ROS Waypoint Tracker Node Started Successfully")

    def UpdateCurrentPose(self, odom_message):

        self.currentPose.header = odom_message.header

        self.currentPose.pose = odom_message.pose.pose

        # print(self.currentPose)

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

        

        #FIX THIS LATER
        s = (now.secs + (now.nsecs*1e-9) - self.path_start_time_offset.secs - self.path_start_time_offset.nsecs)/self.path_duration


        #FOR NOW TEST WITH THIS
        s = 1

        currentPose = self.currentPose.pose

        currentTime = self.currentPose.header.stamp

        self.tf_listener.waitForTransform( '/base_link', '/odom', currentTime, rospy.Duration(1))

        (trans,rot) = self.tf_listener.lookupTransform('/base_link', '/odom', currentTime)

        H_odom2base = tf.transformations.quaternion_matrix(rot)

        H_odom2base[:3,3] = np.array(trans)

        print(H_odom2base)

        #Homogeneous Transformation Matrix for Current Pose
        H_current = tf.transformations.quaternion_matrix([currentPose.orientation.x,currentPose.orientation.y,currentPose.orientation.z,currentPose.orientation.w])
        H_current[:,3] = np.array([currentPose.position.x, currentPose.position.y, currentPose.position.z, 1])

        

        
        theta_traj = self.theta_cubic_params[0]*s**3 + self.theta_cubic_params[1]*s**2 + self.theta_cubic_params[2]*s + self.theta_cubic_params[3]
        x_traj = self.x_cubic_params[0]*s**3 + self.x_cubic_params[1]*s**2 + self.x_cubic_params[2]*s + self.x_cubic_params[3]
        y_traj = self.y_cubic_params[0]*s**3 + self.y_cubic_params[1]*s**2 + self.y_cubic_params[2]*s + self.y_cubic_params[3]

        #Homogeneous Transformation Matrix for Trajectory Pose with respect to base_link
        H_traj = tf.transformations.euler_matrix(0,0,theta_traj)
        H_traj[:,3] = np.array([x_traj, y_traj, 0, 1])


        errorTwist = self.GetTwistError(np.eye(4), H_traj)

        #Calculating the Feed-forward Term
        x_dot  = 3*self.x_cubic_params[0]*s**2 + 2*self.x_cubic_params[1]*s + self.x_cubic_params[2]
        y_dot = 3*self.y_cubic_params[0]*s**2 + 2*self.y_cubic_params[1]*s + self.y_cubic_params[2]
        theta_dot = 3*self.theta_cubic_params[0]*s**2 + 2*self.theta_cubic_params[1]*s + self.theta_cubic_params[2]


        self.twistMessage.linear.x = x_dot + self.Kdx*errorTwist[0] + self.Kpx*(x_traj - H_current[0,3])
        self.twistMessage.linear.y = y_dot + self.Kdy*errorTwist[1] + self.Kpy*(y_traj - H_current[1,3])
        self.twistMessage.linear.z = 0

        self.twistMessage.angular.x = 0
        self.twistMessage.angular.y = 0
        self.twistMessage.angular.z = theta_dot + self.Kdz*errorTwist[5] + self.Kpz*(theta_traj - tf.transformations.euler_from_matrix(H_current)[2])

        #Limit velocities

        if self.twistMessage.linear.x > 0.1:
            self.twistMessage.linear.x = 0.1

        if self.twistMessage.linear.y > 0.1:
            self.twistMessage.linear.y = 0.1
        
        if self.twistMessage.angular.z > 0.1:
            self.twistMessage.angular.z = 0.1

        self.twistPub.publish(self.twistMessage)

    def GetTwistError(self, H_current, H_traj):

        ############ ROS Twist messages are in the base link frame so any trajectory plan twsist computation deom current pose
        ############ to next timestep pose must take place in the base link frame, hence the current frame should be Identity

        # print(H_current) 
        # print(H_current)
        # print(H_traj)

        errorTransform = np.matmul(H_current, H_traj)

        omega, v, theta = self.H2Twist(errorTransform)

        return theta*np.array([v[0], v[1], v[2], omega[0], omega[1], omega[2]])


    def H2Twist(self, H):

        # print("H2Twist Called")

        if np.all(H[0:3,0:3] == np.eye(3)):

            omega = np.asarray([0,0,0])
            if np.linalg.norm(H[:3,3]) == 0:
                v = np.asarray([0,0,0])
            else:
                v = H[:3,3]/np.linalg.norm(H[:3,3])
            theta = np.linalg.norm(H[:3,3])

        else:

            theta, omega = self.Rot2Omega(H[0:3,0:3])

            skew_omega = np.asarray([[0, -omega[2], omega[1]],[omega[2], 0, -omega[0]],[-omega[1], omega[0], 0]])

            G_inv = (1/theta)*np.eye(3) - 0.5*skew_omega + ((1/theta) - 0.5*(1/np.tan(theta/2)))*np.matmul(skew_omega,skew_omega)

            v = np.matmul(G_inv, H[:3,3])
        
        return omega, v, theta

    def Rot2Omega(self, R):

        # print(np.all(R == np.eye(3)))

        if np.all(R == np.eye(3)):
            theta=0
            omega = np.asarray([0,0,0])

        elif np.trace(R) == -1:
            theta = np.pi

            if (1 + R[2,2])>0:
                omega = (1/np.sqrt(2*(1 + R[2,2])))*np.asarray([R[0,2], R[1,2], 1+R[2,2]])

            elif (1 + R[1,1])>0:
                omega = (1/np.sqrt(2*(1 + R[1,1])))*np.asarray([R[0,2], 1 + R[1,1], R[2,1]])

            elif (1 + R[0,0])>0:
                omega = (1/np.sqrt(2*(1 + R[0,0])))*np.asarray([1 + R[0,0], R[1,0], R[2,0]])

        else:
            theta = np.arccos(0.5*(np.trace(R)-1))

            omega = (1/(2*np.sin(theta)))*(R - R.T)

            omega = np.asarray([-omega[1,2],omega[0,2], -omega[0,1]])

        return theta, omega
    
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

################################################### TESTING METHODS ######################################################

    def TestPublishers(self):

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
        
    def TestRot2Omega(self):

        print("Testing Identity Rotation, result should be 0 rotation and no axis:")
        theta, omega = self.Rot2Omega(np.eye(3))
        print("Theta, omega for identity rotation are:")
        print(theta)
        print(omega)
        print()

        print("Testing +90 degree about z-axis rotation, result should be ~1.57 radians and 0,0,1 axis")
        theta, omega = self.Rot2Omega(np.array([[0,-1,0],[1,0,0],[0,0,1]]))
        print(theta)
        print(omega)
        print()

        print("Testing -90 degree about y-axis rotation, result should be -1.57 radians and 0,1,0 axis or 1.57 rad and 0.-1,0 axis")
        theta, omega = self.Rot2Omega(np.array([[0,0,-1],[0,1,0],[1,0,0]]))
        print(theta)
        print(omega)
        print()

    def TestH2Twist(self):

        print("Testing Identity Transformation, result should be 0 rotation and no axis and no velocity:")
        omega, v, theta = self.H2Twist(np.eye(4))
        print(theta)
        print(omega)
        print(v)
        print("")

        print("Testing +90 degree about z-axis rotation, result should be ~1.57 theta scaling radians and 0,0,1 axis and unit positive velocity in x")
        omega, v, theta = self.H2Twist(np.array([[0, -1, 0, 1],[1,0,0, 1],[0,0,1, 0], [0,0,0,1]]))
        print(theta)
        print(omega)
        print(v)
        print("")

        print("Testing -90 degree about y-axis rotation, result should be -1.57 radians and 0,1,0 or axis or 1.57 rad and 0.-1,0 axis and and z positive velocity")
        omega, v, theta = self.H2Twist(np.array([[0,0,-1, -1],[0,1,0, 0],[1,0,0, 1],[0,0,0, 1]]))
        print(theta)
        print(omega)
        print(v)
        print("")

    def TestGetTwistError(self):

        print("Testing Identity Transformation, result should be a zero twist:")
        twist = self.GetTwistError(np.eye(4), np.eye(4))
        print(twist)
        print("")

        print("Testing +90 degree about z-axis rotation, result should be ~1.57 theta scaling radians and 0,0,1 axis and unit positive velocity in x")
        twist = self.GetTwistError(np.array([[1, 0, 0, 0],[0,1,0, 0],[0,0,1, 0], [0,0,0,1]]), np.array([[0, -1, 0, -1],[1,0,0, 1],[0,0,1, 0], [0,0,0,1]]))
        print(twist)
        print("")


    def TestGenerateTwists(self):

        print("Testing Identity Transformation, result should be a zero twist:")
        self.GenerateTwists()






navPlan = NavPlan()
navPlan.StartNode()
navPlan.UpdatePathParams()


#########################Testing Rot2Omega
# navPlan.TestRot2Omega()

#########################Testing H2Twist
# navPlan.TestH2Twist()

#########################Testing GetTwistError
# navPlan.TestGetTwistError()

#########################Generate Twists

while not rospy.is_shutdown():
    navPlan.TestGenerateTwists()


rospy.spin()