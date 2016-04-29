#!/usr/bin/env python
## -------PID controller to reach April tags -------------

#Used for testing
#Subscriber: AR Tag /tagdetections
#publisher: v and w (subscibed by uni_to_diff)

#defines
import rospy
import roslib; roslib.load_manifest('gBot')
from std_msgs.msg import String
from apriltags_ros.msg import (
    AprilTagDetectionArray,
    AprilTagDetection
)
from geometry_msgs.msg import (
    PoseArray,
    PoseStamped,
    Pose,
    Point,
    Quaternion,
    Twist,
)

from sensor_msgs.msg import JointState
import math
import numpy as np

class tagDetect:
	def __init__( self ):
		#define tag which finds if tag is detected or not
		self.tag = 0
		self.position = 0
    
		#Subscribe to ar tag detection node
    		rospy.Subscriber("/tag_detections",AprilTagDetectionArray, self.callback)

		#Current location
		self.x = 0.0
		self.y = 0.0
		self.theta = 0.0
		self.pose = np.array((self.x, self.y))

		#PID gains
		self.Kp = 0.01
		self.Kd = 0
		self.Ki = 0

		#error storage elements
		self.E_k = 0
		self.e_k_1 = 0

		#PID outputs
		self.v = 0.1
		self.w = 0

		#Robot parameters
		#R is radius of wheel in mm
		self.R = 30
		#L is length of robot in mm
		self.L = 185

		#Control params
		self.d_stop = 0.05
		self.rc = False


	def callback(self, data):
		#april tag topic subscriber callback
    		msg = data.detections
    		i = 0

    		if msg:

			rospy.loginfo('Tag detected')
			self.tag = 1
    			self.position = msg[i].pose.pose.position

    			rospy.loginfo("Point Position: [ %f, %f, %f ]"%(self.position.x, self.position.y, self.position.z))
			rospy.loginfo(self.tag)

    		else:
			rospy.loginfo('No tag detected!!')
			self.tag = 0
			self.position = None 
			rospy.loginfo(self.tag)
	
	def pid(self, data):
		#Function that does a z control over the heading for the ground vehicle
		#Uses z from the ar tag and tries to reach it
		
		#Calculate heading angle to goal
		u_x = data.x - self.x
		u_y = data.y - self.y 
		theta_g = math.atan2(u_y, u_x)
		
		#Calculate error in heading
		e_k = theta_g - self.theta
		e_k = math.atan2(math.sin(e_k), math.cos(e_k))

		#Calculate PID parameters
		e_P = e_k
		e_I = self.E_k + e_k
		e_D = self.e_k_1 - e_k

		#The controlled angular velocity
		self.w = self.Kp*e_P + self.Ki*e_I + self.Kd*e_D
		
		#Updates
		self.E_k = e_I
		self.e_k_1 = e_k

		#Print statements for debugging
		rospy.loginfo('PID called, w is: ')
		rospy.loginfo(self.w)

		goal = np.array((data.x, data.y))
		rospy.loginfo('distance from goal: ')
		rospy.loginfo(np.linalg.norm(self.pose-goal))

	
	##Translate from unicycle to differential drive dynamics
	def uni_to_diff(self):

		self.vel_r = (2*self.v + self.w*self.L)/(2*self.R)
		self.vel_l = (2*self.v - self.w*self.L)/(2*self.R)

		if self.vel_r > 0.2: 	
			self.vel_r = 0.2
		if self.vel_r < -0.2:
			self.vel_r = -0.2
		if self.vel_l > 0.2:
			self.vel_l = 0.2
		if self.vel_l < -0.2 :
			self.vel_l = -0.2
	
		rospy.loginfo('velocities calculated, right velocity is: ')
		rospy.loginfo(self.vel_r)
		rospy.loginfo('left velocity is: ')
		rospy.loginfo(self.vel_l)
		#publish the calculated velocities

		
	def at_goal(self, data):
		goal = np.array((data.x, data.y))
		if (np.linalg.norm(self.pose-goal) < self.d_stop):
			self.rc = True

def detect():
	    	
	rospy.init_node('go_to_tag', anonymous=True)


	r = rospy.Rate(10)
	obj = tagDetect()
	
	while not rospy.is_shutdown():

		if obj.tag == 1:
			
			obj.at_goal(obj.position)
			if obj.rc:
				rospy.loginfo('Goal reached')

			else:
				rospy.loginfo('yay, tag found')
				#calculate w for reaching goal
				obj.pid(obj.position)
				#Convert from uni cycle to diff drive
				obj.uni_to_diff()
		
		elif obj.tag == 0:
			rospy.loginfo('Oops, no tag')
		else:
			rospy.loginfo('something is wrong')

		r.sleep()
		



if __name__ == '__main__':
 	try:
         	detect()
        except rospy.ROSInterruptException:
        	pass

