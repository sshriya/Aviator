#!/usr/bin/env python
#launch ardrone and try to find tags.
#If tags found, go to tag

##TAG pose: front +ve y, back -ve y
#right -ve x, left +ve x
import rospy
import roslib; roslib.load_manifest('drone_autonomy')
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
)
import numpy as np
        
class tagDetect:
	def __init__( self ):
		#define tag which finds if tag is detected or not
		self.tag = 0

		#Measurements
        	self.position = np.zeros((3,1))
        	self.velocity = np.zeros((3,1))

		#Robot's current state
        	self.position_c = np.zeros((3,1))
        	self.velocity_c = np.zeros((3,1))

    		# publish commands (send to quadrotor)
    		self.pub_takeoff = rospy.Publisher('/ardrone/takeoff', Empty)
    		self.pub_land = rospy.Publisher('/ardrone/land', Empty)
    		self.pub_reset = rospy.Publisher('/ardrone/reset', Empty)

		#Subscribe to ar tag detection node
    		rospy.Subscriber("/tag_detections",AprilTagDetectionArray, self.callback)

		#Variable used to test 
		self.fly = 1
    
        	# xy control gains
        	Kp_xy = 1 # xy proportional
        	Kd_xy = 0.05 # xy differential
        
        	# height control gains
        	Kp_z  = 1 # z proportional
        	Kd_z  = 0.01 # z differential
        
        	self.Kp = np.array([[Kp_xy, Kp_xy, Kp_z]]).T
        	self.Kd = np.array([[Kd_xy, Kd_xy, Kd_z]]).T

		print('Initilization completed!')

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


	def safeLand(self):
		print('safe landing!!')
		rospy.loginfo('testing 123')
	
	def takeOff(self):
		rospy.loginfo('take off maybe??')
		self.pub_takeoff.publish(Empty())
		rospy.sleep(10)

    	def go2tag(self, pos, pos_g, vel, vel_g):
        	'''
        	:param t: time since simulation start
        	:param dt: time since last call to measurement_callback
        	:param state: State - current quadrotor position and velocity computed from noisy measurements
        	:param state_desired: State - desired quadrotor position and velocity
        	:return - xyz velocity control signal represented as 3x1 numpy array
        	'''
       
        	error_pos = pos_g - pos
        	error_vel = vel_g - vel 
        	u = self.Kp*error_pos + self.Kd*error_vel
		print 'The control velocity vector is'
		rospy.loginfo(u)
        
        	return u


def detect():
	    	
	rospy.init_node('findTagLand', anonymous=True)

	#Switch to bottom camera
	#rosservice call --wait /ardrone/togglecam

	r = rospy.Rate(5)
	obj = tagDetect()


	while not rospy.is_shutdown():
		#Start with simple takeoff
		while obj.fly:
			print('Take off!!')
			#obj.pub_takeoff.publish(Empty())
	
			#If tag is found, go to tag
			if obj.tag == 1:
				rospy.loginfo('yay, tag found')
    				#obj.pub_land.publish(Empty())
				self.u_command  = obj.go2tag(obj.position_c, obj.position, obj.velocity_c, obj.velocity)
				odj.fly = 0

			elif obj.tag == 0:
				rospy.loginfo('Oops, no tag')
			else:
				rospy.loginfo('something is wrong')

			r.sleep()
		r.sleep()
		


if __name__ == '__main__':
 	try:
         	detect()
        except rospy.ROSInterruptException:
        	pass

