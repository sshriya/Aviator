#!/usr/bin/env python
import rospy
import roslib; roslib.load_manifest('ardrone_python')
from ardrone_autonomy.msg import Navdata
import time
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import numpy as np
from math import sin, cos

class Publish_odometry:
    def __init__( self ):
        self.nav_sub = rospy.Subscriber( "ardrone/navdata", Navdata, self.callback_navdata )
        self.pub = rospy.Publisher( "drone_controller/odometry", Twist )

	#define msg for publishing
	self.msg = Twist()

        self.vx = self.vy = self.vz = self.ax = self.ay = self.az = self.rotZ = 0.0
        self.last_time = None

	self.position = np.array([[0], [0]])
        
    def measurement_callback(self):
        '''
        :param t: time since simulation start
        :param dt: time since last call to measurement_callback
        :param navdata: measurements of the quadrotor
        '''
        if self.last_time == None:
            self.last_time = rospy.Time.now()
            dt = 0.0
        else:
            time = rospy.Time.now()
            dt = ( time - self.last_time ).to_sec()
            self.last_time = time

        #update self.position by integrating measurements contained in navdata
        R = np.array([[cos(self.rotZ), -sin(self.rotZ)], [sin(self.rotZ), cos(self.rotZ)]])
        Vt = np.dot(R, np.array([[self.vx],[self.vy]]))
        
        self.position = self.position + Vt*dt
        print self.position

    
    def callback_navdata( self, data ):
        self.vx = data.vx/1e3
        self.vy = data.vy/1e3
        self.vz = data.vz/1e3
	self.rotZ = data.rotZ
        t = data.header.stamp.to_sec()


def main():
  rospy.init_node('odometry', anonymous=True)

  odometry = Publish_odometry()
  r = rospy.Rate(10)
	

  try:
      while not rospy.is_shutdown():
          odometry.measurement_callback()

	  odometry.msg.linear.x = odometry.position[0]
          odometry.msg.linear.y = odometry.position[1]
          #odometry.pub.publish(odometry.msg)
          r.sleep()

  except KeyboardInterrupt:
    print "Shutting down"


if __name__ == '__main__':
    main()
    
