#! /usr/bin/env python3
import rospy
import pygame
import math as m
from sensor_msgs.msg import Joy

WIDTH, HEIGHT = 90, 90
WIN = pygame.display.set_mode((WIDTH, HEIGHT))

pygame.joystick.init()
joysticks = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
print(joysticks)
speed = 0
FPS = 60
con = 3
button_X = 0
button_A = 1
button_B = 2
print("init")
def main():
    # clock = pygame.time.Clock()
    run = True
    speed = 0
    speed_time = 0.03
    rospy.init_node("Joystick")
    pub = rospy.Publisher("/joy0", Joy, queue_size= 1)
    Data = Joy()
    rate = rospy.Rate(100)
    def pub_once(Joy):
        good = True
        while good:
            connections = pub.get_num_connections()
            if connections > 0:
                pub.publish(Joy)
                good = False
                break
            else:
                rate.sleep()
    while run:
        # clock.tick(FPS)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
        x_pos = pygame.joystick.Joystick(con).get_axis(0)
        y_pos = pygame.joystick.Joystick(con).get_axis(1)
        B_A = pygame.joystick.Joystick(con).get_button(button_A)
        B_B = pygame.joystick.Joystick(con).get_button(button_B)
        B_X = pygame.joystick.Joystick(con).get_button(button_X)
        if B_A:
            speed = speed + speed_time
            if speed > 1:
                speed = 1.0
        elif B_B:
            speed = speed - speed_time
            if speed < -1:
                speed = -1.0
        else:
            if speed > 0:
                speed = speed - speed_time
                if speed < 0:
                    speed = 0
            elif speed < 0:
                speed = speed + speed_time
                if speed > 0:
                    speed = 0
        if B_X:
            speed = 0
        if x_pos < 0.2 and x_pos > -0.2:
            x_pos = 0.0
        if x_pos > 0.75:
            x_pos = 1
        if x_pos < -0.75:
            x_pos = -1
        if y_pos < 0.2 and y_pos > -0.2:
            y_pos = 0.0
        if y_pos > 0.75:
            y_pos = 1
        if y_pos < -0.75:
            y_pos = -1
        x_float = "{:.2f}".format(x_pos)
        y_float = "{:.2f}".format(y_pos)
        speed_value = "{:.2f}".format(speed)
        angle = m.atan2(y_pos,x_pos)
        deg = angle * -180/m.pi
        pos = [x_float, y_float]
        deg = "{:.2f}".format(deg)
        Data.axes = [float(y_float), float(x_float)]
        Data.buttons = [B_A, B_B, B_X]
        print("Angle = ", deg, " Pos = ", pos, " Speed = ", speed_value)
        pub_once(Data)
    pygame.quit()

if __name__ == "__main__":
    main()