#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64

rospy.init_node("move_edu_rover")
pub = rospy.Publisher("/edu_rover/left_back_knuckle_wheel_controller/command", Float64, queue_size= 1)
