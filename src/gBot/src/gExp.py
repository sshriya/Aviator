#!/usr/bin/env python
#Start random exploration by the bot and try to find tags.
#If tags found, go to tag
#Working copy
#Date 6th Dec, 2015
#Author Shriya Shah

##TAG pose: front +ve y, back -ve y
#right -ve x, left +ve x
import rospy
import roslib; roslib.load_manifest('gBot')
from std_msgs.msg import Bool

from std_msgs.msg import Empty

from geometry_msgs.msg import (
    PoseArray,
    PoseStamped,
    Pose,
    Point,
    Quaternion,
    Twist,
    Vector3
)

from sensor_msgs.msg import JointState
import numpy as np
import math
from numpy import interp
import time

#import pid
class exploration:
	def __init__( self ):
		#define tag which finds if tag is detected or not from subscriber
		self.tag = 0

		#Goal positions
		self.x_g = 0
		self.y_g = 0
		self.z_g = 0

		#Current state
		self.x = 0
		self.y = 0
		self.theta = 0.0
		self.pose = np.array((self.x, self.y))

		#Velocities
		self.v = 0.15
		self.w = 0 #(math.pi)/4 #for testing

		#maximum velocities
		self.max_rpm = 200 #130
		self.min_rpm = 30
		self.max_vel = (self.max_rpm*math.pi)/60;
        	self.min_vel = (self.min_rpm*math.pi)/60;

		#PID gains
		self.Kp = 4
		self.Kd = 0.01
		self.Ki = 0.01

		#error storage elements
		self.E_k = 0
		self.e_k_1 = 0

		#Controller timing
		self.last_time = 0

		#Minimum distance to goal
		self.x_min = 0.1
		self.y_min = 0.1

		#Robot parameters
		self.R = .030 #R is radius of wheel in m
		self.L = .195 #L is length between wheels of robot in m

		#state machine variables
		self.motorStatus = 1 #1 exploration , 0 stop
		self.startPID = 0

		#Variable used to test 
		self.start = 1
		self.finish = 0
		self.goal = 0

		#Subscribe to ar tag detection node
    		rospy.Subscriber("/gBot/tagFlag",Bool, self.tagCb)
		rospy.Subscriber("/gBot/tagPose", Vector3, self.poseCb)
    
		#Define publishers
        	self.p1 = rospy.Publisher('/gBot/left_wheel/cmd', JointState)
		self.p2 = rospy.Publisher('/gBot/right_wheel/cmd', JointState)
		self.p3 = rospy.Publisher('/gBot/pidFlag', Bool)

		#Publishining message
        	#Message for left_wheel
		self.leftW = JointState()
        	self.leftW.name = ["left_wheel"]
        	self.leftW.position = [0.0]
        	self.leftW.velocity = [0.0]

        	#Message for right_wheel
		self.rightW = JointState()
        	self.rightW.name = ["right_wheel"]
        	self.rightW.position = [0.0]
        	self.rightW.velocity = [0.0]

		#Message to notify the PID to start operation		
		self.msg3 = Bool()
		print('Initilization completed!')

	#!!!!!!!!Callback routines!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	def tagCb(self, data):
		#Read if tag seen 
		self.tag = data.data
		#if tag see, stop the robot and generate PID flag
		if self.tag:
			self.motorStatus = 0 #stop

		else:
			self.motorStatus = 1 #Exploration
	
	def poseCb(self, data):
		if not (data.x == 0):  
			self.x_g = data.x
		if not (data.y == 0):
			self.y_g = data.y
		#self.z_g = data.z

	def moveForward(self, vel_r, vel_l):
		print("Move Forward")
		#Both wheel same velocities
		self.leftW.velocity = [vel_l]
		self.rightW.velocity = [vel_r]

	def sign(self, x):
		if x > 0:
			sig = 1
		elif x < 0:
			sig  = -1
		elif (x == 0):
			sig = 0
		
		return sig

	#!!!!!!!!!!PUblishing message !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	def publishMesg(self):
		self.leftW.header.stamp = rospy.Time.now()
     		self.p1.publish(self.leftW)
		self.rightW.header.stamp = rospy.Time.now()
               	self.p2.publish(self.rightW)
		#self.msg3.data = self.startPID
		#self.p3.publish(self.msg3)

	#**************Controller design********************************
	def atGoal(self, x_g, y_g):	
		error_pos_x = x_g - self.x
		error_pos_y = y_g - self.y
		print("Error in x is: ")
		print(error_pos_x)
		print("Error is y is:")
		print(error_pos_y)	
		
		if (-self.x_min < error_pos_x and error_pos_x < self.x_min) and (error_pos_y < self.y_min and error_pos_y > -self.y_min):
			#Stop robot
			print("Goal reached")
			self.goal = 1 #Goal reached
			self.u_x = 0
			self.u_y = 0
			return 1
		else:
			return 0


	def go_to_goal(self, x_g, y_g):
		
        	#if goal not reached
		if not self.atGoal(x_g, y_g):
			#Calculate heading angle to goal
			u_x = x_g - self.x
			u_y = y_g - self.y 
			theta_g = math.atan2(u_y, u_x)
			print("Theta_g is: ")
			print(theta_g)
		
			#Calculate error in heading
			e_k = theta_g - self.theta
			e_k = math.atan2(math.sin(e_k), math.cos(e_k))
			print("Error is heading: ")
			print(e_k)
			
			#calculate time
            		time = rospy.get_time()
            		dt = ( time - self.last_time )
            		self.last_time = time

			#Calculate PID parameters
			e_P = e_k
			e_I = self.E_k + e_k*dt
			e_D = (e_k - self.e_k_1)/dt
	
			#The controlled angular velocity
			w = self.Kp*e_P + self.Ki*e_I + self.Kd*e_D
			v = self.v

			#Updates
			self.E_k = e_I
			self.e_k_1 = e_k

			#Print statements for debugging
			rospy.loginfo('The required value of heading, w is: ')
			rospy.loginfo(w)

			goal = np.array((x_g, y_g))
			rospy.loginfo('distance from goal: ')
			rospy.loginfo(np.linalg.norm(self.pose-goal))

			return (v, w)

	def go_to_angle(self, theta_g):

		print("Extecuting go to angle controller")
		print("Theta_g is: ")
		print(theta_g)
		
		#Calculate error in heading
		e_k = theta_g - self.theta
		e_k_2 = math.atan2(math.sin(e_k), math.cos(e_k))
		print("Error is heading: ")
		print(e_k_2)

		#Calculate PID parameters
		e_P = e_k_2
	
		#The controlled angular velocity
		w = self.Kp*e_P 
		v = 0.15
		#Print statements for debugging
		rospy.loginfo('The required value if heading, w is: ')
		rospy.loginfo(w)
		return (v, w)

	#*********Differential drive robot*********************************
	def uni_to_diff(self, v, w):
		
		vel_r_d = (2*v + w*self.L)/(2*self.R)
		vel_l_d = (2*v - w*self.L)/(2*self.R)

		return (vel_r_d, vel_l_d)
 	
	def diff_to_uni(self,r,l):

            	v_d = self.R/2*(r+l)
            	w_d = self.R/self.L*(r-l)

		return (v_d, w_d)		

	def ensure_w(self, v, w):
            
		# This function ensures that w is respected as best as possible
        	# by scaling v.
            
        	vel_max = self.max_vel
        	vel_min = self.min_vel
           
        	if (abs(v) > 0):
                	# 1. Limit v,w to be possible in the range [vel_min, vel_max]
                	# (avoid stalling or exceeding motor limits)
                	v_lim = max(min(abs(v), (self.R/2)*(2*vel_max)), (self.R/2)*(2*vel_min))
                	w_lim = max(min(abs(w), (self.R/self.L)*(vel_max-vel_min)), 0)
                
                	# 2. Compute the desired curvature of the robot's motion
                	
                	[vel_r_d, vel_l_d] = self.uni_to_diff(v_lim, w_lim)
                	
                	# 3. Find the max and min vel_r/vel_l
                	vel_rl_max = max(vel_r_d, vel_l_d)
                	vel_rl_min = min(vel_r_d, vel_l_d)
                
                	# 4. Shift vel_r and vel_l if they exceed max/min vel
                	if (vel_rl_max > vel_max):
                    		vel_r = vel_r_d - (vel_rl_max-vel_max)
                    		vel_l = vel_l_d - (vel_rl_max-vel_max)
                	elif (vel_rl_min < vel_min):
                    		vel_r = vel_r_d + (vel_min-vel_rl_min)
                    		vel_l = vel_l_d + (vel_min-vel_rl_min)
                	else:
                    		vel_r = vel_r_d
                    		vel_l = vel_l_d
			
                	# 5. Fix signs (Always either both positive or negative)
                	[v_shift, w_shift] = self.diff_to_uni(vel_r, vel_l)
			v_sign = self.sign(v)
			w_sign = self.sign(w)
			
                	v = (v_sign)*(v_shift)
                	w = (w_sign)*(w_shift)
                	
            	else:
                	# Robot is stationary, so we can either not rotate, or
                	# rotate with some minimum/maximum angular velocity
                	w_min = self.R/self.L*(2*vel_min)
                	w_max = self.R/self.L*(2*vel_max)

			w_sign = self.sign(w)
                
                	if abs(w) > w_min:
                    		w = (w_sign)*(max(min(abs(w), w_max), w_min))
                	else:
                    		w = 0;
     		
            	[vel_r, vel_l] = self.uni_to_diff(v, w)

		print("Corrected velocities for right and left wheel is: ")
		print(vel_r)
		print(vel_l)
		return (vel_r, vel_l)
		

	def set_pwm(self, vel_r, vel_l):
		# actuator hardware limits            
            	vel_r = max(min(vel_r, self.max_vel), -self.max_vel)
            	vel_l = max(min(vel_l, self.max_vel), -self.max_vel)
            
            	vel_r = vel_r*(abs(vel_r) >= self.min_vel)
            	vel_l = vel_l*(abs(vel_l) >= self.min_vel)

		if vel_r > vel_l:
			#Robot is turning left
			vel_r = vel_r
			vel_l = -vel_l
		if vel_r < vel_l:
			#robot is turning right
			vel_r = -vel_r
			vel_l = vel_l

            	#final pwm values
		if vel_r > 0:
            		right_wheel_speed = interp(vel_r, [self.min_vel, self.max_vel], [50, 200])
		elif vel_r < 0:
			right_wheel_speed = interp(vel_r, [-self.min_vel, -self.max_vel], [-50, -100])
		if vel_l > 0:
            		left_wheel_speed = interp(vel_l, [self.min_vel, self.max_vel], [50, 200])
		elif vel_l < 0:
            		left_wheel_speed = interp(vel_l, [-self.min_vel, -self.max_vel], [-50, -100])

		print("PWM values for right and left wheel are:")
		print(right_wheel_speed)
		print(left_wheel_speed)
		return (right_wheel_speed, left_wheel_speed)


def main():
	    	
	rospy.init_node('gexploration', anonymous=True)
	
	obj = exploration()
	rospy.on_shutdown(onShutDown)

	print('Starting exploration!!')

	while not rospy.is_shutdown():
		#system starts in exploration mode
		if obj.motorStatus:
			obj.moveForward(80, 80)
			obj.publishMesg()
		#If tag found, stop and go to tag
		elif obj.motorStatus == 0:
			#obj.stop()
			while not obj.goal:
				
				print("Tag found, going towards tag")
				
				[v, w] = obj.go_to_goal(obj.x_g, obj.y_g)

				#Output to controller goes to ensure w
				[vel_r, vel_l] = obj.ensure_w(v, w)

				#Output of ensure w goes to set pwm
				[rightW, leftW] = obj.set_pwm(vel_r, vel_l)

				rospy.sleep(.1)
				#publishing motor velocities
				obj.leftW.velocity = [leftW]
				obj.rightW.velocity = [rightW]
				obj.publishMesg()
				
			#goal reached
			break

		rospy.sleep(.1)	

	
def onShutDown():
		print("Shutting down")
        	p1 = rospy.Publisher('/gBot/left_wheel/cmd', JointState)
		p2 = rospy.Publisher('/gBot/right_wheel/cmd', JointState)
		m1 = JointState()
		m2 = JointState()
		m1.velocity = [0]
		m2.velocity = [0]
		p1.publish(m1)
		p2.publish(m2)

if __name__ == '__main__':
 	try:
         	main()
        except rospy.ROSInterruptException:
        	pass

