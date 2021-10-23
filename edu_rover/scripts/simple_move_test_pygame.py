#! /usr/bin/env python3

import pygame
import rospy
import math as ma
from std_msgs.msg import Float64
import time
def main():
    WIDTH, HEIGHT = 100,100
    WIN = pygame.display.set_mode((WIDTH,HEIGHT))
    servo_angle = 0
    servo = 0
    speed = 0
    run = True
    mode = "none"    
    
    rospy.init_node("pygame_control", anonymous = True)
    pub_base_left_front_knuckle = rospy.Publisher("/edu_rover/base_left_front_knuckle_controller/command", Float64, queue_size= 1)
    pub_base_left_back_knuckle =  rospy.Publisher("/edu_rover/base_left_back_knuckle_controller/command",  Float64, queue_size= 1)
    pub_base_right_front_knuckle =rospy.Publisher("/edu_rover/base_right_front_knuckle_controller/command",Float64, queue_size= 1)
    pub_base_right_back_knuckle = rospy.Publisher("/edu_rover/base_right_back_knuckle_controller/command", Float64, queue_size= 1)

    pub_left_front_knuckle_wheel = rospy.Publisher("/edu_rover/left_front_knuckle_wheel_controller/command", Float64, queue_size= 1)
    pub_left_back_knuckle_wheel =  rospy.Publisher("/edu_rover/left_front_knuckle_wheel_controller/command", Float64, queue_size= 1)
    pub_right_front_knuckle_wheel =rospy.Publisher("/edu_rover/right_front_knuckle_wheel_controller/command",Float64, queue_size= 1)
    pub_right_back_knuckle_wheel = rospy.Publisher("/edu_rover/right_back_knuckle_wheel_controller/command", Float64, queue_size= 1)

    #rate = rospy.Rate()
    
    def deg_to_rad(angle):
        pi = 355/113
        rad_angle = angle*pi/180
        return rad_angle

    def servo_angle():

        if key_pressed[pygame.K_a]:
            servo_angle = 30
            knuckle_left_angle = deg_to_rad(servo_angle)
            knuckle_right_angle = deg_to_rad(servo_angle)
            servo = [knuckle_left_angle, knuckle_right_angle]

        elif key_pressed[pygame.K_d]:
            servo_angle = -30
            knuckle_left_angle = deg_to_rad(servo_angle)
            knuckle_right_angle = deg_to_rad(servo_angle)
            servo = [knuckle_left_angle, knuckle_right_angle]

        else:
            servo_angle = 0.0
            knuckle_left_angle = deg_to_rad(servo_angle)
            knuckle_right_angle = deg_to_rad(servo_angle)
            servo = [knuckle_left_angle, knuckle_right_angle]
        return servo

    
    while run:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
        key_pressed = pygame.key.get_pressed()
        if key_pressed[pygame.K_1]:
            mode = "forward_drive"

        if key_pressed[pygame.K_2]:
            mode = "sideways_drive"
        
        if key_pressed[pygame.K_3]:
            mode = "tight"

        if mode == "forward_drive":
            servo_1 = servo_angle()[0]
            servo_2 = servo_angle()[1]
            servo = servo_1
            # L = 0.1
            # dist = 0.15
            # R = m.atan2(dist, abs(servo_1))

            if key_pressed[pygame.K_w]:
                speed = 5.0

            elif key_pressed[pygame.K_s]:
                speed = -5.0

            # diff_drive_radius = dist/m.tan(abs(servo_1))
            # vel = speed
            # omega = vel/diff_drive_radius
            # front_wheel_left_vel = 
            # back_wheel_left_vel = omega*(R - L/2)
            # back_wheel_right_vel = omega*(R + L/2)
            pub_base_left_front_knuckle.publish(servo_1)
            pub_base_left_back_knuckle.publish(0.0)
            pub_base_right_front_knuckle.publish(servo_2)
            pub_base_right_back_knuckle.publish(0.0)
            
            pub_left_front_knuckle_wheel.publish(speed)
            pub_left_back_knuckle_wheel.publish(speed)
            pub_right_front_knuckle_wheel.publish(speed)
            pub_right_back_knuckle_wheel.publish(speed)

        if mode == "sideways_drive":
            if key_pressed[pygame.K_w]:
                speed = 5.0
                pub_left_front_knuckle_wheel.publish(speed)
                pub_left_back_knuckle_wheel.publish(-speed)
                pub_right_front_knuckle_wheel.publish(-speed)
                pub_right_back_knuckle_wheel.publish(speed)

            elif key_pressed[pygame.K_s]:
                speed = -5.0
                pub_left_front_knuckle_wheel.publish(speed)
                pub_left_back_knuckle_wheel.publish(-speed)
                pub_right_front_knuckle_wheel.publish(-speed)
                pub_right_back_knuckle_wheel.publish(speed)

            else:
                speed = 0.0
                pub_left_front_knuckle_wheel.publish(speed)
                pub_left_back_knuckle_wheel.publish(-speed)
                pub_right_front_knuckle_wheel.publish(-speed)
                pub_right_back_knuckle_wheel.publish(speed)

            pub_base_left_front_knuckle.publish(-deg_to_rad(90))
            pub_base_left_back_knuckle.publish(deg_to_rad(90))
            pub_base_right_front_knuckle.publish(deg_to_rad(90))
            pub_base_right_back_knuckle.publish(-deg_to_rad(90))

        else:
            pub_base_left_front_knuckle.publish(0.0)
            pub_base_left_back_knuckle.publish(0.0)
            pub_base_right_front_knuckle.publish(0.0)
            pub_base_right_back_knuckle.publish(0.0)
            
            pub_left_front_knuckle_wheel.publish(0.0)
            pub_left_back_knuckle_wheel.publish(0.0)
            pub_right_front_knuckle_wheel.publish(0.0)
            pub_right_back_knuckle_wheel.publish(0.0)
        print("speed = ", speed, "angle = ", servo*180/(355/113))
        #rate.sleep()

    pygame.quit()
            
if __name__ == "__main__":
    main()
