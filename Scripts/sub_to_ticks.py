#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32, Float64MultiArray


# current_time = rospy.Time.now()
# last_time = rospy.Time.now()

# r = rospy.Rate(1.0)
unit = rospy.get_param("/unit")


import math
from math import sin, cos, pi

import rospy
# import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3    

def publish_odom(vx, vy,vth):
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    # current_time = rospy.Time.now()

    # compute odometry in a typical way given the velocities of the robot

    # x += delta_x
    # y += delta_y
    # th += delta_th

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    # odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    # first, we'll publish the transform over tf
    # odom_broadcaster.sendTransform(
    #     (x, y, 0.),
    #     odom_quat,
    #     current_time,
    #     "base_link",
    #     "odom"
    # )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    # odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    # odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    # publish the message
    odom_pub.publish(odom)

    # last_time = current_time
    # r.sleep()


class listener():

    def __init__(self):
        

        self.right_tick = 0
        self.left_tick = 0
        # In ROS, nodes are uniquely named. If two nodes with the same
        # name are launched, the previous one is kicked off. The
        # anonymous=True flag means that rospy will choose a unique
        # name for our 'listener' node so that multiple listeners can
        # run simultaneously.
        rospy.init_node('publisher', anonymous=True)

        # rospy.Subscriber("/right_tick", Int32, self.callback_r)
        # rospy.Subscriber("/left_tick", Int32, self.callback_l)
        rospy.Subscriber(unit + "/velocities", Float64MultiArray, self.callback_v)



        # spin() simply keeps python from exiting until this node is stopped
    def callback_l(self,msg):
        self.left_tick = msg

    def callback_r(self,msg):
        self.right_tick = msg
        publish_odom(self.left_tick, self.right_tick)

    def callback_v(self, msg):
        vx= msg.data[0]
        vy= msg.data[1]
        vth = msg.data[2]
        # print(vx,vy,vth)
        publish_odom(vx,vy,vth)

    def loop(self):
        rospy.spin()

if __name__ == '__main__':
    my_sub = listener()
    my_sub.loop()

    
