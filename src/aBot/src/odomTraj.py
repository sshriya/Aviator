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
from nav_msgs.msg import Odometry
from ardrone_autonomy.msg import Navdata

class tagDetect:
	def __init__( self ):

		#Start system in hover mode
		self.hovering = 1 #0 means stitch to go-to-tag mode

		self.firstRun = 1

		#define tag which finds if tag is detected or not
		self.tag = 0
		#to find goal
		self.goal = 0

		self.state = 1

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
		
		#Square trajectory
		self.iX1 = 1
		self.iY1 = 0

		self.iX2 = 1
		self.iY2 = 1

		self.iX3  = 0
		self.iY3 = 1

		self.iX4 = 0
		self.iY4 = 0

		#Odometry readings
		self.currentX = 0
		self.currentY = 0
		self.currentVelx = 0
		self.currentVely = 0
        	#self.vx = self.vy = self.vz = self.ax = self.ay = self.az = self.rotZ = 0.0
		
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
		rospy.Subscriber("/ardrone/odometry", Odometry, self.currentPoseCb)
		#rospy.Subscriber("drone_controller/odometry", Twist, self.myOdomCb)
		#rospy.Subscriber( "ardrone/navdata", Navdata, self.callback_navdata )

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

	def currentPoseCb(self, data):	
		self.currentX = data.pose.pose.position.x
		self.currentY = data.pose.pose.position.y
		self.currentVelx = data.twist.twist.linear.x
		self.currentVely = data.twist.twist.linear.y


		'''
		if self.firstRun:
			self.iX = self.currentX
			self.iY = self.currentY
			self.firstRun  = 0
		'''
	
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
			
	def holdPose(self, initialX, initialY):
		#THIS ONE WORKS GOOD ENOUGH
		#Kp = 0.1
		#Kd = 0.1

		Kp = 0.005
		Kd = 0.008
		'''
		print("Initial position to be held:")
		print(initialX)
		print(initialY)
	
		print("Current pose of drone:")
		print(self.currentX)
		print(self.currentY)
		
		print("Current velocity of drone:")
		print(self.currentVelx)
		print(self.currentVely)
		'''
		vel_x = Kp*(initialX - self.currentX) + Kd*(0 - self.currentVelx)
		vel_y = Kp*(initialY - self.currentY ) + Kd*(0 - self.currentVely)
		print("velocities required to hold position is: ")
		print(vel_x)
		print(vel_y)
		return (vel_x, vel_y)	

	def limitU(self, u_x, u_y):

		if u_x < -0.6:
			u_x = -0.6
		if u_x > 0.6:
			u_x = 0.6
		if u_y < -0.6:
			u_y = -0.6
		if u_y > 0.6:
			u_y = 0.6

		return (u_x, u_y)

	

def detect():
	    	
	rospy.init_node('findTagLand', anonymous=True)
	rospy.on_shutdown(onShutDown)

	#Switch to bottom camera
	#rosservice call --wait /ardrone/togglecam

	#r = rospy.Rate(5)
	obj = tagDetect()
	
	

	while not rospy.is_shutdown():

		if obj.hovering:

			if obj.state == 1:
			#move forward
				print("Move forward1")
				print("initial position in detect")
				print(obj.iX1)
				print(obj.iY1)
				[vx, vy] = obj.holdPose(obj.iX1, obj.iY1)
				[ux_lim, uy_lim] = obj.limitU(vx, vy)
				
				print("Current pose of drone:")
				print(obj.currentX)
				print(obj.currentY)
		
				print("Current velocity of drone:")
				print(vx)
				print(vy)

				print("limited u's are: ")
				print(ux_lim)
				print(uy_lim)
			
			
				obj.pub_velocity.publish(Twist(Vector3(ux_lim,uy_lim,0),Vector3(0,0,0)))
				print("Error: ")
				print(obj.iX1 - obj.currentX)
				if (obj.iX1 - obj.currentX) < 0.2:
					obj.state = 2
					print("Switching to turn right")
 
				#rospy.sleep(.01)

			elif obj.state == 2:
			#Turn right
				print("Turn right1")
				print("initial position in detect")
				print(obj.iX2)
				print(obj.iY2)
				[vx, vy] = obj.holdPose(obj.iX2, obj.iY2)
				[ux_lim, uy_lim] = obj.limitU(vx, vy)
				
				print("Current pose of drone:")
				print(obj.currentX)
				print(obj.currentY)
		
				print("Current velocity of drone:")
				print(vx)
				print(vy)

				print("limited u's are: ")
				print(ux_lim)
				print(uy_lim)
			
			
				obj.pub_velocity.publish(Twist(Vector3(ux_lim,uy_lim,0),Vector3(0,0,0)))
				print("Error: ")
				print(obj.iY2 - obj.currentY)
				if (obj.iY2 - obj.currentY) < 0.1:
					obj.state = 5
					print("Switching to move forward")
 
				#rospy.sleep(.01)

			elif obj.state == 3:
			#Move straight
				print("Move straight2")
				print("initial position in detect")
				print(obj.iX3)
				print(obj.iY3)
				[vx, vy] = obj.holdPose(obj.iX3, obj.iY3)
				[ux_lim, uy_lim] = obj.limitU(vx, vy)
				
				print("Current pose of drone:")
				print(obj.currentX)
				print(obj.currentY)
		
				print("Current velocity of drone:")
				print(vx)
				print(vy)

				print("limited u's are: ")
				print(ux_lim)
				print(uy_lim)
			
			
				obj.pub_velocity.publish(Twist(Vector3(ux_lim,uy_lim,0),Vector3(0,0,0)))
				print("Error: ")
				print(obj.iX3 - obj.currentX)
				if (obj.iX3 - obj.currentX) < 0.1:
					obj.state = 4
					print("turn right again")
 
				#rospy.sleep(.01)

			elif obj.state == 4:
			#Turn right
				print("Turn right2")
				print("initial position in detect")
				print(obj.iX4)
				print(obj.iY4)
				[vx, vy] = obj.holdPose(obj.iX4, obj.iY4)
				[ux_lim, uy_lim] = obj.limitU(vx, vy)
				
				print("Current pose of drone:")
				print(obj.currentX)
				print(obj.currentY)
		
				print("Current velocity of drone:")
				print(vx)
				print(vy)

				print("limited u's are: ")
				print(ux_lim)
				print(uy_lim)
			
			
				obj.pub_velocity.publish(Twist(Vector3(ux_lim,uy_lim,0),Vector3(0,0,0)))
				print("Error: ")
				print(obj.iY4 - obj.currentY)
				if (obj.iY4 - obj.currentY) < 0.1:
					obj.state = 5
					print("switching to landing")
 
				#rospy.sleep(.01)

			elif obj.state == 5:
			#Mission complete
				print("Mission completed, intiate landing")
				obj.pub_land.publish(Empty())	

			else:
				print("Something went wrong")


		#if tag found, switch to go to tag mode
		else:

			print("Tag seen, going to tag..")

			#keep on going to goal till goal found
			while not obj.goal:
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
				rospy.sleep(.01)
			
			#if at goal
			if obj.goal: 
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

