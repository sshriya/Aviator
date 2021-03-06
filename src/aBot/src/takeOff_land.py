#!/usr/bin/env python
import rospy
import roslib; roslib.load_manifest('drone_autonomy')
from std_msgs.msg import Empty

if __name__ == '__main__':
	    	
	rospy.init_node('takeOff_land', anonymous=True)
    
    	# publish commands (send to quadrotor)
    	pub_takeoff = rospy.Publisher('/ardrone/takeoff', Empty)
    	pub_land = rospy.Publisher('/ardrone/land', Empty)
    	pub_reset = rospy.Publisher('/ardrone/reset', Empty)

	print("ready!")
    	rospy.sleep(1.0)
    
    	print("takeoff..")
    	pub_takeoff.publish(Empty())
    	rospy.sleep(5.0)
    
    	print("land..")
    	pub_land.publish(Empty())
    
    	print("done!")

