#!/usr/bin/env python
#launch ardrone and try to find tags.
#If tags found, initiate landing

##TAG pose: front +ve y, back -ve y
#right -ve x, left +ve x
import rospy
import roslib; roslib.load_manifest('aBot')
from std_msgs.msg import Bool

from std_msgs.msg import Empty

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
    Vector3
)

from sensor_msgs.msg import JointState
import numpy as np

class exploration:
	def __init__( self ):
		#define tag which finds if tag is detected or not
		self.tagFound = 0

		#Subscribe to ar tag detection node
    		rospy.Subscriber("/aBot/tagFlag",Bool, self.tagCb)
		rospy.Subscriber("/aBot/tagPose", Vector3, self.poseCb)
    

		#Variable used to test 
		self.fly = 1
		self.goal = 0

		#Goal location
		self.x_g = 0
		self.y_g = 0
		self.z_g = 0

		#Current loacation
		self.x = 0
		self.y = 0
		self.z = 0

		#Publisher nodes
    		self.pub_velocity = rospy.Publisher('/cmd_vel', Twist)
        	self.pub_takeoff = rospy.Publisher('/ardrone/takeoff', Empty)
        	self.pub_land = rospy.Publisher('/ardrone/land', Empty)
    		self.pub_reset = rospy.Publisher('/ardrone/reset', Empty)

		print('Initilization completed!')


	def poseCb(self, data):
		if not (data.x == 0):  
			self.x_g = data.x
		if not (data.y == 0):
			self.y_g = data.y
		if not (data.z == 0):
			self.z_g = data.z

	def tagCb(self, data):
		#Read if tag seen 
		self.tagFound = data.data
		#if tag see, stop the robot and generate PID flag
		'''
		if self.tag:
			self.fly = 0 #stop

		else:
			self.fly = 1 #Exploration
		'''

	def takeoff(self):
		print("Takeoff")
		self.pub_takeoff.publish(Empty())
		rospy.sleep(3)
		#Sleep and wait for 5 seconds
	
	def land(self):
    		print("land..")
    		self.pub_land.publish(Empty())
    		rospy.sleep(3)
    		print("done!")

	def flyForward(self, ):
		print("Flying forward")
        	self.pub_velocity.publish(Twist(Vector3(0.05,0,0),Vector3(0,0,0)))
    		rospy.sleep(7.0)

	def hover(self):
		print("stop..")
    		self.pub_velocity.publish(Twist(Vector3(0,0,0),Vector3(0,0,0)))
    		rospy.sleep(3.0)
	
	def turnRight(self):
		print("Turn right")
		self.pub_velocity.publish(Twist(Vector3(0,0,0),Vector3(0,0,1)))
    		rospy.sleep(3.0)


def main():
	    	
	rospy.init_node('exploration', anonymous=True)

	#Switch to bottom camera
	#rosservice call --wait /ardrone/togglecam

	#r = rospy.Rate(5)
	obj = exploration()

	while not rospy.is_shutdown():
		#system starts in exploration mode
		if obj.fly:
			print('Take off!!')
			print("Searching for tags")
			obj.takeoff()
			obj.fly = 0 #Ensure that it takes off only once 
			
		#if tag found, hover and wait for some time and then go to the goal		
		elif obj.fly == 0:
			if obj.tagFound: 
				print("Tag visible, located at")
				print(obj.x_g)
				print(obj.y_g)	
				print(obj.z_g)
				print("Landing")
				obj.land()
				rospy.sleep(1)
			else: 
				print("Searching for tags")
				obj.hover()
			'''	
			while not obj.goal:
			#do exploration while tag not found
				print("Tag seen, time to go to goal")
    				print("land..")
    				rospy.sleep(1)
				#obj.goal = 1 #goal reached 

				break
			'''
		rospy.sleep(.1)

		#rospy.loginfo("Exploration completed...")

def onShutDown():
		print("Shutting down")
		p1 = rospy.Publisher('/ardrone/reset', Empty)
		p1.publish(Empty())
    		rospy.sleep(3)
		'''
        	p1 = rospy.Publisher('/gBot/left_wheel/cmd', JointState)
		p2 = rospy.Publisher('/gBot/right_wheel/cmd', JointState)
		m1 = JointState()
		m2 = JointState()
		m1.velocity = [0]
		m2.velocity = [0]
		p1.publish(m1)
		p2.publish(m2)
		'''
if __name__ == '__main__':
 	try:
         	main()
        except rospy.ROSInterruptException:
        	pass

