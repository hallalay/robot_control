#!/usr/bin/env python

from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #     

import rospy
# in this example we try to obtain linear
# and angular velocity related information.
# So we import Twist
from geometry_msgs.msg import Twist

import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers

from math import pi

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

x=0

radius=0.055
width=0.13

unit = rospy.get_param("/unit")


BP.set_motor_power(BP.PORT_C, BP.MOTOR_FLOAT)                          # float motor D
BP.set_motor_power(BP.PORT_B, BP.MOTOR_FLOAT)

def count_dps(x,z):
	l_rps = (x-(z*width/2))/radius
	r_rps = (x+(z*width/2))/radius

	l_dps = l_rps*180/pi
	r_dps = r_rps*180/pi

	return l_dps, r_dps

def callback(msg):

	# rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
	# rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))

	x = msg.linear.x
	z = msg.angular.z

	l_dps, r_dps = count_dps(x,z)

	BP.set_motor_dps(BP.PORT_B, l_dps)             # set the target speed for motor A in Degrees Per Second
	BP.set_motor_dps(BP.PORT_C, r_dps)
	time.sleep(0.02)
	
def listener():
	# In ROS, nodes are uniquely named. If two nodes with the same
	# name are launched, the previous one is kicked off. The
	# anonymous=True flag means that rospy will choose a unique
	# name for our 'listener' node so that multiple listeners can
	# run simultaneously.

	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber(unit + "/cmd_vel", Twist, callback)
 
	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()
 
if __name__ == '__main__':
	try:
		listener()
	except KeyboardInterrupt:
		rospy.sleep(0.1)
		BP.reset_all() 

