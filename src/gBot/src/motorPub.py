#!/usr/bin/env python
##Publishes joint states for all the two motors on 2 separate joint state topics
#***********used with arduino on AutoBot*******************************
#For testing of wiring and connections
#Velocity command between -255 to 255
##---------------------------------------------------------------------
import rospy
from sensor_msgs.msg import JointState

def motor_pub():
        p1 = rospy.Publisher('/gBot/left_wheel/cmd', JointState)
	p2 = rospy.Publisher('/gBot/right_wheel/cmd', JointState)
       
        rospy.init_node('motorPub', anonymous=True)
 	rospy.on_shutdown(motor_stop)
        #Message for left_wheel
	msg1 = JointState()
        msg1.name = ["left_wheel"]
        msg1.position = [0.0]
        msg1.velocity = [-100]

        #Message for right_wheel
	msg2 = JointState()
        msg2.name = ["right_wheel"]
        msg2.position = [0.0]
        msg2.velocity = [-100]
	

        while not rospy.is_shutdown():

                msg1.header.stamp = rospy.Time.now()
                p1.publish(msg1)
		msg2.header.stamp = rospy.Time.now()
                p2.publish(msg2)
		rospy.loginfo('Left Motor velocity')
		rospy.loginfo(msg1.velocity)
		rospy.loginfo('Right Motor velocity')
		rospy.loginfo(msg2.velocity)
                rospy.sleep(0.1)


def motor_stop():
        p1 = rospy.Publisher('/gBot/left_wheel/cmd', JointState)
	p2 = rospy.Publisher('/gBot/right_wheel/cmd', JointState)

	rospy.loginfo('Motor velocity publishing stopped')
        #Message for left_wheel
	msg1 = JointState()
        msg1.name = ["left_wheel"]
        msg1.position = [0.0]
        msg1.velocity = [0.0]

        #Message for right_wheel
	msg2 = JointState()
        msg2.name = ["right_wheel"]
        msg2.position = [0.0]
        msg2.velocity = [0.0]
	p1.publish(msg1)
	p2.publish(msg2)
	rospy.sleep(1)
	
	

if __name__ == '__main__':
        try:
                motor_pub()

        except rospy.ROSInterruptException:
                pass



