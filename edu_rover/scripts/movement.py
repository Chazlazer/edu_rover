#! /usr/bin/env python3

import rospy
import time
from std_msgs.msg import Float64
from geometry_msgs import Twist
from sensor_msgs.msg import JointState

class edu_rover_mover(object):

    def __init__(self):
        rospy.init_node('Edu_Rover_Joint_Mover', anonymous = True)
        rospy.loginfo("Edu_Rover JointMover Initalising...")

        self.pub_base_left_front_knuckle = rospy.Publisher("/edu_rover/base_left_front_knuckle_controller/command", Float64, queue_size= 1)
        self.pub_base_left_back_knuckle =  rospy.Publisher("/edu_rover/base_left_back_knuckle_controller/command",  Float64, queue_size= 1)
        self.pub_base_right_front_knuckle =rospy.Publisher("/edu_rover/base_right_front_knuckle_controller/command",Float64, queue_size= 1)
        self.pub_base_right_back_knuckle = rospy.Publisher("/edu_rover/base_right_back_knuckle_controller/command", Float64, queue_size= 1)

        self.pub_left_front_knuckle_wheel = rospy.Publisher("/edu_rover/left_front_knuckle_wheel_controller/command", Float64, queue_size= 1)
        self.pub_left_back_knuckle_wheel =  rospy.Publisher("/edu_rover/left_front_knuckle_wheel_controller/command", Float64, queue_size= 1)
        self.pub_right_front_knuckle_wheel =rospy.Publisher("/edu_rover/right_front_knuckle_wheel_controller/command",Float64, queue_size= 1)
        self.pub_right_back_knuckle_wheel = rospy.Publisher("/edu_rover/right_back_knuckle_wheel_controller/command", Float64, queue_size= 1)

        joint_states_topic_name = "/edu_rover/joint_states"

        rospy.Subscriber(joint_states_topic_name, JointState, self.edu_rover_joints_callback)
        edu_rover_joints_data = None
        rate = rospy.Rate(2)

        while edu_rover_data is None:
            try:
                edu_rover_joints_data = rospy.wait_for_message(joint_states_topic_name, JointState, timeout = 5)
            except:
                rospy.logwarn("Time out ", + str(joint_states_topic_name))
                pass
            rate.sleep()
        self.edu_rover_joint_dictionary = dict(zip(edu_rover_joints_data.name, edu_rover_joints_data.position))

    def move_edu_rover_all_joints(self, left_front_knuckle_angle, left_back_knuckle_angle, right_front_knuckle_angle, right_back_knuckle_angle,
                                        left_front_wheel)