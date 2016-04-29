#!/usr/bin/env python
#launch ardrone and try to find tags.
#If tags found, initiate landing

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

class tagDetect:
	def __init__( self ):
		#define tag which finds if tag is detected or not
		self.tag = 0
		self.position = 0
    
    		# publish commands (send to quadrotor)
    		self.pub_takeoff = rospy.Publisher('/ardrone/takeoff', Empty)
    		self.pub_land = rospy.Publisher('/ardrone/land', Empty)
    		self.pub_reset = rospy.Publisher('/ardrone/reset', Empty)

		#Subscribe to ar tag detection node
    		rospy.Subscriber("/tag_detections",AprilTagDetectionArray, self.callback)

		print('Initilization completed!')

		#Variable used to test 
		self.fly = 1


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

def detect():
	    	
	rospy.init_node('findTagLand', anonymous=True)

	#Switch to bottom camera
	#rosservice call --wait /ardrone/togglecam

	r = rospy.Rate(5)
	obj = tagDetect()


	while not rospy.is_shutdown():

		while obj.fly:
			print('Take off!!')
			obj.pub_takeoff.publish(Empty())
	
			if obj.tag == 1:
				rospy.loginfo('yay, tag found')
    				obj.pub_land.publish(Empty())
				obj.fly = 0

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

