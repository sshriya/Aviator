#!/usr/bin/env python
#launch ardrone and try to find tags.
#If tags found, initiate landing

##TAG pose: front +ve y, back -ve y
#right -ve x, left +ve x
import rospy
import roslib; roslib.load_manifest('aBot')
from std_msgs.msg import Empty
from std_msgs.msg import Bool
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

class tagDetect:
	def __init__( self ):

		#Start system in hover mode
		self.hovering = 1 #0 means stitch to go-to-tag mode

		#define tag which finds if tag is detected or not
		self.tag = 0
		#to find goal
		self.goal = 0

		#messages to be published
		self.msg = Empty()

		#goal position
		self.current_position = np.zeros((3,1))
        	self.goal_position = np.zeros((3,1))
		self.R = np.array([[0, 1], [-1, 0]])

		#Goal location
		self.x_g = 0
		self.y_g = 0
		self.z_g = 0

		#Current loacation
		self.x = 0
		self.y = 0
		self.z = 0
		
		#Control commands
		self.u_x = 0
		self.u_y = 0
		self.U = np.array([[0], [0]])

		#Minimum distance to goal
		self.x_min = 0.1
		self.y_min = 0.1

        	# xy control gains
        	self.Kp_xy = 0.2 # xy proportional
        	self.Kd_xy = 1 # xy differential
        
        	# height control gains
        	self.Kp_z  = 1 # z proportional
        	self.Kd_z  = 0 # z differential
        
        	#self.Kp = np.array([[Kp_xy, Kp_xy]]).T
        	#self.Kd = np.array([[Kd_xy, Kd_xy]]).T

    
    		# publish commands (send to quadrotor)
    		self.pub_takeoff = rospy.Publisher('/ardrone/takeoff', Empty)
    		self.pub_land = rospy.Publisher('/ardrone/land', Empty)
    		self.pub_reset = rospy.Publisher('/ardrone/reset', Empty)
		self.pub_reset = rospy.Publisher('/ardrone/reset', Empty)
		self.pub_velocity = rospy.Publisher('/cmd_vel', Twist)

		#Subscribe to ar tag detection node
    		rospy.Subscriber("/aBot/tagFlag",Bool, self.tagCb)
		rospy.Subscriber("/aBot/tagPose", Vector3, self.poseCb)

		print('Initilization completed!')

		#Variable used to test 
		self.fly = 1

	def poseCb(self, data):
		#record data only when tag is seen
		if not (data.x == 0):  
			self.x_g = data.x
		if not (data.y == 0):
			self.y_g = data.y
		if not (data.z == 0):
			self.z_g = data.z
		print("goal is at: ")
		print(self.x_g)
		print(self.y_g)

	def tagCb(self, data):
		#Read if tag seen 
		self.tag = data.data
		#if tag seen, 
		if self.tag:
			self.hovering = 0 #tag seen. go to controller mode
		else:	
			self.hovering = 1 #No tag, switch to hover mode again
		
	
	def takeOff(self):
		print("Move Forward")
		#Both wheel same velocities
		self.msg = Empty()

	#!!!!!!!!!!PUblishing message !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	def publishMesg(self):
     		self.pub_takeoff.publish(self.msg)
		#self.msg3.data = self.startPID
		#self.p3.publish(self.msg3)

	#**************Controller design********************************
	def atGoal(self, x_g, y_g):	
		error_pos_x = x_g - self.x
		error_pos_y = y_g - self.y
		#error_pos_z = z_g - self.z
		print("Error in x is: ")
		print(error_pos_x)
		print("Error is y is:")
		print(error_pos_y)
		#print("Error is z is:")
		#print(error_pos_z)	
		
		if (-self.x_min < error_pos_x and error_pos_x < self.x_min) and (error_pos_y < self.y_min and error_pos_y > -self.y_min):
			#Stop robot
			print("Goal reached")
			self.goal = 1 #Goal reached
			self.u_x = 0
			self.u_y = 0
			self.u_z = 0
			return 1
		else:
			return 0
	
	def checkQuad(self, x_g, y_g):
		if x_g < 0 and y_g > 0:
			#in Quad 1 , [Ux, Uy] = (+,-)
			Ux = self.u_x
			Uy = self.u_y
		if x_g > 0 and y_g > 0:
			#in Quad 2 , [Ux, Uy] = (+,+)
			Ux = self.u_x
			Uy = -1*self.u_y
		if x_g > 0 and y_g < 0:
			#in Quad 3 , [Ux, Uy] = (-, - )
			Ux = -1*self.u_x
			Uy = -1*self.u_y
		if x_g < 0 and y_g < 0:
			#in Quad 1 , [Ux, Uy] = (+,+)
			Ux = -1*self.u_x
			Uy = self.u_y
		return (Ux, Uy)

	def transformGoal(self, x_g, y_g):
		X_goal = np.array([[x_g], [y_g]])
		print("X_goal is:")
		print(X_goal)
		X_transformed = np.dot(self.R, X_goal)
		print("Transformed goal is at: ")
		x_g = X_transformed[0]
		y_g = X_transformed[1]
		return (x_g, y_g)



	def go_to_tag(self, x_g, y_g):
		if not self.atGoal(x_g, y_g):
			print ('go to tag controller called...calculated velocities are: ')
			print('Goal positions are at x_g, y_g in camera frame of reference ')
			print(x_g)
			print(y_g)
			[X, Y] = self.transformGoal(x_g, y_g)
			print('Goal positions are at x_g, y_g in robot frame of reference ')
			print(X)
			print(Y)
			
			self.u_x = self.Kp_xy*(X - self.x)
			self.u_y = self.Kp_xy*(Y - self.y)
			#self.u_z = self.Kp_z*(z_g - self.z)
			#print("Controls before quad correction")
			#print(self.u_x)
			#print(self.u_y)
			#[self.u_x, self.u_y] = self.checkQuad(x_g, y_g)

			#u = self.Kp * (state_desired.position - state.position) + self.Kd * (state_desired.velocity - state.velocity)
			
	
	def limitU(self, u_x, u_y):

		if u_x < -0.8:
			u_x = -0.8
		if u_x > 0.8:
			u_x = 0.8
		if u_y < -0.8:
			u_y = -0.8
		if u_y > 0.8:
			u_y = 0.8

		return (u_x, u_y)
	

def detect():
	    	
	rospy.init_node('findTagLand', anonymous=True)
	rospy.on_shutdown(onShutDown)

	#Switch to bottom camera
	#rosservice call --wait /ardrone/togglecam

	#r = rospy.Rate(5)
	obj = tagDetect()


	while not rospy.is_shutdown():
	
		#start in hover mode
		if obj.hovering: 
			print("Hovering and looking around...")
			obj.pub_velocity.publish(Twist(Vector3(0,0,0.05),Vector3(0,0,0)))
    			#rospy.sleep(2.0)
		
		#if tag found, switch to go to tag mode
		elif obj.hovering == 0:
			print("Tag seen, going to tag..")

			#keep on going to goal till goal found
			if not obj.goal:
				rospy.loginfo('yay, tag found')
				#go to tag
				obj.go_to_tag(obj.x_g, obj.y_g)
				print("Velocities ux and uy caluculated from go-to-tag: ")
				print(obj.u_x)
				print(obj.u_y)
				[ux_lim, uy_lim] = obj.limitU(obj.u_x, obj.u_y)
				print("limited u's are: ")
				print(ux_lim)
				print(uy_lim)
				obj.pub_velocity.publish(Twist(Vector3(ux_lim,uy_lim,0),Vector3(0,0,0)))
				rospy.sleep(.1)
			
			#if at goal
			elif obj.goal: 
				#initiate landing
				print("Goal reached, intiate landing")
				obj.pub_land.publish(Empty())	
				break
			#break
		rospy.sleep(.1)
		#obj.start = 0

def onShutDown():
		print("Shutting down")
        	pub_land = rospy.Publisher('/ardrone/land', Empty)
    		pub_reset = rospy.Publisher('/ardrone/reset', Empty)
		pub_land.publish(Empty())
		#pub_reset.publish(Empty())


if __name__ == '__main__':
 	try:
         	detect()
        except rospy.ROSInterruptException:
        	pass

