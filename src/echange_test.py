#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

def callback(msg):
	print(msg)

rospy.init_node("env_node")

rate = rospy.Rate(2)
obs = Float32()
obs.data = 10.0

pub = rospy.Publisher('/counter', Float32)
sub = rospy.Subscriber('/decision', Float32, callback)
while not rospy.is_shutdown():
	pub.publish(obs)
rospy.spin()
