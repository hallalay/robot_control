#!/usr/bin/env python
# license removed for brevity
from __future__ import print_function # use python 3 syntax but make it compatible with python 2
from __future__ import division       #     

import rospy
from std_msgs.msg import Int32, Float64MultiArray

import time     # import the time library for the sleep function
import brickpi3 # import the BrickPi3 drivers

BP = brickpi3.BrickPi3() # Create an instance of the BrickPi3 class. BP will be the BrickPi3 object.

radius = 0.055
base = 0.13
ticks_per_meter = 1000
meters_per_tick = 1/ticks_per_meter
old_r_tick = 0.0
old_l_tick = 0.0
old_time = 0.0


def get_speeds(r_tick,l_tick):
	global old_l_tick, old_r_tick, old_time
	now = time.time()
	diff = now-old_time

	r_tick_diff = (r_tick-old_r_tick)
	l_tick_diff = (l_tick-old_l_tick)



	vR = (r_tick_diff/diff)*meters_per_tick
	vL = (l_tick_diff/diff)*meters_per_tick
	
	vx = (vR + vL)/2.0
	vy = 0.0
	vth = (vR-vL)/base
	
	old_time=now
	old_l_tick = l_tick
	old_r_tick = r_tick

	return vx, vy, vth


def talker():
	pub_r = rospy.Publisher('right_tick', Int32, queue_size=10)
	pub_l = rospy.Publisher('left_tick', Int32, queue_size=10)
	pub_v = rospy.Publisher('velocities', Float64MultiArray, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10) # 10hz

	try:
		BP.offset_motor_encoder(BP.PORT_B, BP.get_motor_encoder(BP.PORT_B)) # reset encoder A
		BP.offset_motor_encoder(BP.PORT_C, BP.get_motor_encoder(BP.PORT_C)) # reset encoder D
	except IOError as error:
		print(error)

	while not rospy.is_shutdown():
		
		r_tick = BP.get_motor_encoder(BP.PORT_C) 
		l_tick = BP.get_motor_encoder(BP.PORT_B) 

		vx, vy, vth = get_speeds(r_tick, l_tick)
		velocities = Float64MultiArray()
		velocities.data = [vx, vy, vth]

		pub_r.publish(r_tick)
		pub_l.publish(l_tick)
		pub_v.publish(velocities)
		rate.sleep()


if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass