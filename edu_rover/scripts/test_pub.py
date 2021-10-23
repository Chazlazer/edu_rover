#! /usr/bin/env python3

import rospy
from std_msgs.msg import Float64

def knuckle_pub():
    rospy.init_node("left_front_knuckle_joint_publisher")
    left_front_knuckle_pub = rospy.Publisher("/edu_rover/base_left_front_knuckle_controller/command", Float64, queue_size= 1)
    left_back_knuckle_pub = rospy.Publisher("/edu_rover/base_left_back_knuckle_controller/command", Float64, queue_size= 1)
    right_front_knuckle_pub = rospy.Publisher("/edu_rover/base_right_front_knuckle_controller/command", Float64, queue_size= 1)
    right_back_knuckle_pub = rospy.Publisher("/edu_rover/base_right_back_knuckle_controller/command", Float64, queue_size= 1)
    rate = rospy.Rate(50)
    angle_deg = 0.0
    pi = 355/113
    while not rospy.is_shutdown():
        angle_rad = angle_deg * pi/180
        left_front_knuckle_pub.publish(-angle_rad)
        left_back_knuckle_pub.publish(angle_rad)
        right_front_knuckle_pub.publish(angle_rad)
        right_back_knuckle_pub.publish(-angle_rad)
        rate.sleep()

if __name__ == "__main__":
    try:
        knuckle_pub()
    except rospy.ROSInterruptException:
        pass