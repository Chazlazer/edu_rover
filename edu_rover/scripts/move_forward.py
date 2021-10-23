#!/usr/bin/env python

import rospy
import time
from math import pi, sin, cos, acos
from std_msgs import Float64
from sensor_msgs import JointState
from geometry_msgs import Twist

class edu_rover_move(object):
    
    def __init__(self):
        rospy.loginfo("edu_rover JointMover Initializing...")

        self.pub_base_left_front_knuckle = rospy.Publisher("/edu_rover/base_left_front_knuckle_controller/command", Float64, queue_size= 1)
        self.pub_base_left_back_knuckle =  rospy.Publisher("/edu_rover/base_left_back_knuckle_controller/command",  Float64, queue_size= 1)
        self.pub_base_right_front_knuckle =rospy.Publisher("/edu_rover/base_right_front_knuckle_controller/command",Float64, queue_size= 1)
        self.pub_base_right_back_knuckle = rospy.Publisher("/edu_rover/base_right_back_knuckle_controller/command", Float64, queue_size= 1)

        self.pub_left_front_knuckle_wheel = rospy.Publisher("/edu_rover/left_front_knuckle_wheel_controller/command",  Float64, queue_size= 1)
        self.pub_left_back_knuckle_wheel = rospy.Publisher("/edu_rover/left_front_knuckle_wheel_controller/command",   Float64, queue_size= 1)
        self.pub_right_front_knuckle_wheel = rospy.Publisher("/edu_rover/right_front_knuckle_wheel_controller/command",Float64, queue_size= 1)
        self.pub_right_back_knuckle_wheel = rospy.Publisher("/edu_rover/right_back_knuckle_wheel_controller/command",  Float64, queue_size= 1)

        self.twist_value = Twist()

        rospy.Subscriber("/cmd_vel", Twist, self._cmd_vel_callback)

        self.joint_states_topic_name = "/edu_rover/joint_states"

        edu_rover_joints_data = self._check_joint_states_ready()
        

        if gurdy_joints_data is not None:
            self.edu_rover_joint_dictionary = dict(zip(edu_rover_joints_data.name,edu_rover_joints_data.position))
            rospy.Subscriber(self.joint_states_topic_name, JointState,self.edu_rover_joints_callback)
    
    def _check_joint_states_ready(self):
        self.joint_states = None
        rospy.logdebug("Waiting for "+str(self.joint_states_topic_name)+" tobe READY...")
        while self.joint_states is None and not rospy.is_shutdown():
            try:
                self.joint_states = rospy.wait_for_message(self.joint_states_topic_name, JointState, timeout=5.0)
                rospy.logdebug("Current "+str(self.joint_states_topic_name)+"READY=>")
            except:
                rospy.logerr("Current "+str(self.joint_states_topic_name)+"not ready yet, retrying")
        return self.joint_states

    

