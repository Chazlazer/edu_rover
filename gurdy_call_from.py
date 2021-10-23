#!/usr/bin/env python
import rospy
import time
from math import pi, sin, cos, acos
import random
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
class gurdyJointMover(object):
    def __init__(self):
        rospy.loginfo("Gurdy JointMover Initialising...")

        self.pub_upperlegM1_yaw_joint_position = rospy.Publisher('/gurdy/upperlegM1_yaw_joint_position_controller/command',Float64,queue_size=1)
        self.pub_upperlegM2_yaw_joint_position = rospy.Publisher('/gurdy/upperlegM2_yaw_joint_position_conroller/command',Float64,queue_size=1)
        self.pub_upperlegM3_yaw_joint_position = rospy.Publisher('/gurdy/upperlegM3_yaw_joint_position_controller/command',Float64,queue_size=1)
        self.pub_upperlegM1_joint_position = rospy.Publisher('/gurdy/head_upperlegM1_joint_position_controller/command',Float64,queue_size=1)
        self.pub_upperlegM2_joint_position = rospy.Publisher('/gurdy/head_upperlegM2_joint_position_controller/command',Float64,queue_size=1)
        self.pub_upperlegM3_joint_position = rospy.Publisher('/gurdy/head_upperlegM3_joint_position_controller/command',Float64,queue_size=1)
        self.pub_lowerlegM1_joint_position = rospy.Publisher('/gurdy/upperlegM1_lowerlegM1_joint_position_controller/command',Float64,queue_size=1)
        self.pub_lowerlegM2_joint_position = rospy.Publisher('/gurdy/upperlegM2_lowerlegM2_joint_position_controller/command',Float64,queue_size=1)
        self.pub_lowerlegM3_joint_position = rospy.Publisher('/gurdy/upperlegM3_lowerlegM3_joint_position_controller/command',Float64,queue_size=1)
        self.pub_headarm_yaw_joint_position = rospy.Publisher('/gurdy/head_arm_yaw_joint_position_controller/command',Float64,queue_size=1)
        self.pub_headarm_joint_position = rospy.Publisher('/gurdy/head_arm_joint_position_controller/command',Float64,queue_size=1)
        self.pub_headforearm_joint_position = rospy.Publisher('/gurdy/head_forearm_joint_position_controller/command',Float64,queue_size=1)
        # Add prismatic joints
        self.basefoot_peg_M1_basefoot_peg_M1_joint_joint_position = rospy.Publisher('/gurdy/basefoot_peg_M1_basefoot_peg_M1_joint_joint_position_controller/command',Float64,queue_size=1)
        self.basefoot_peg_M2_basefoot_peg_M2_joint_joint_position = rospy.Publisher('/gurdy/basefoot_peg_M2_basefoot_peg_M2_joint_joint_position_controller/command',Float64,queue_size=1)
        self.basefoot_peg_M3_basefoot_peg_M3_joint_joint_position = rospy.Publisher('/gurdy/basefoot_peg_M3_basefoot_peg_M3_joint_joint_position_controller/command',Float64,queue_size=1)
        
        self.twist_value = Twist()

        rospy.Subscriber("/cmd_vel", Twist, self._cmd_vel_callback)

        self.joint_states_topic_name = "/gurdy/joint_states"

        gurdy_joints_data = self._check_joint_states_ready()

        if gurdy_joints_data is not None:
            self.gurdy_joint_dictionary = dict(zip(gurdy_joints_data.name,gurdy_joints_data.position))
            rospy.Subscriber(self.joint_states_topic_name, JointState,self.gurdy_joints_callback)

    
    
    
    def move_gurdy_all_joints(self, upperlegM1_angle, upperlegM2_angle, upperlegM3_angle, 
    lowerlegM1_value, lowerlegM2_value ,lowerlegM3_value, headarm_value, headforearm_value, 
    upperlegM1_yaw_angle, upperlegM2_yaw_angle,upperlegM3_yaw_angle, headarm_yaw_value, 
    basefoot_peg_M1_value, basefoot_peg_M2_value, basefoot_peg_M3_value):
        upperlegM2_yaw = Float64()
        upperlegM2_yaw.data = upperlegM2_yaw_angle
        upperlegM3_yaw = Float64()
        upperlegM3_yaw.data = upperlegM3_yaw_angle
        upperlegM1 = Float64()
        upperlegM1.data = upperlegM1_angle
        upperlegM2 = Float64()
        upperlegM2.data = upperlegM2_angle
        upperlegM3 = Float64()
        upperlegM3.data = upperlegM3_angle
        lowerlegM1 = Float64()
        lowerlegM1.data = lowerlegM1_value
        lowerlegM2 = Float64()
        lowerlegM2.data = lowerlegM2_value
        lowerlegM3 = Float64()
        lowerlegM3.data = lowerlegM3_value
        headarm_yaw = Float64()
        headarm_yaw.data = headarm_yaw_value
        headarm = Float64()
        headarm.data = headarm_value
        headforearm = Float64()
        headforearm.data = headforearm_value
        self.gurdy_move_ony_m1_yaw(upperlegM1_yaw_angle)
        self.pub_upperlegM2_yaw_joint_position.publish(upperlegM2_yaw)
        self.pub_upperlegM3_yaw_joint_position.publish(upperlegM3_yaw)
        self.pub_upperlegM1_joint_position.publish(upperlegM1)
        self.pub_upperlegM2_joint_position.publish(upperlegM2)
        self.pub_upperlegM3_joint_position.publish(upperlegM3)
        self.pub_lowerlegM1_joint_position.publish(lowerlegM1)
        self.pub_lowerlegM2_joint_position.publish(lowerlegM2)
        self.pub_lowerlegM3_joint_position.publish(lowerlegM3)
        self.pub_headarm_yaw_joint_position.publish(headarm_yaw)
        self.pub_headarm_joint_position.publish(headarm)
        self.pub_headforearm_joint_position.publish(headforearm)
        self.gurdy_move_ony_primsatic(basefoot_peg_M1_value,
        basefoot_peg_M2_value, basefoot_peg_M3_value)

    def gurdy_move_ony_m1_yaw(self, angle):
        """
        Moves only the primatic joints
        """
        upperlegM1_yaw = Float64()
        upperlegM1_yaw.data = angle
        self.pub_upperlegM1_yaw_joint_position.publish(upperlegM1_yaw)

    def gurdy_move_ony_primsatic(self, foot_m1, foot_m2, foot_m3):
        """
        Moves only the primatic joints
        """
        basefoot_peg_M1 = Float64()
        basefoot_peg_M1.data = foot_m1
        basefoot_peg_M2 = Float64()
        basefoot_peg_M2.data = foot_m2
        basefoot_peg_M3 = Float64()
        basefoot_peg_M3.data = foot_m3
        self.basefoot_peg_M1_basefoot_peg_M1_joint_joint_position.publish(basefoot_peg_M1)
        self.basefoot_peg_M2_basefoot_peg_M2_joint_joint_position.publish(basefoot_peg_M2)
        self.basefoot_peg_M3_basefoot_peg_M3_joint_joint_position.publish(basefoot_peg_M3)

    def gurdy_joints_callback(self, msg):
        """
        sensor_msgs/JointState
        std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
        string[] name
        float64[] position
        float64[] velocity
        float64[] effort
        :param msg:
        :return:
        """
        self.gurdy_joint_dictionary = dict(zip(msg.name, msg.position))

    def convert_angle_to_unitary(self, angle):
        """
        Removes complete revolutions from angle and converts to positive
        equivalent
        if the angle is negative
        :param angle: Has to be in radians
        :return:
        """
        # Convert to angle between [0,360)
        complete_rev = 2 * pi
        mod_angle = int(angle / complete_rev)
        clean_angle = angle - mod_angle * complete_rev
        # Convert Negative angles to their corresponding positive values
        if clean_angle < 0:
            clean_angle += 2 * pi
        return clean_angle

    def assertAlmostEqualAngles(self, x, y,):
        c2 = (sin(x) - sin(y)) ** 2 + (cos(x) - cos(y)) ** 2
        angle_diff = acos((2.0 - c2) / 2.0)
        return angle_diff

    def gurdy_check_continuous_joint_value(self, joint_name, value, error=0.1):
        """
        Check the joint by name 'base_waist_joint', 'body_head_joint',
        'waist_body_joint is near the value given
        We have to convert the joint values removing whole revolutions and
        converting negative versions
        of the same angle
        :param joint_name:
        :param value:
        :param error: In radians
        :return:
        """
        joint_reading = self.gurdy_joint_dictionary.get(joint_name)
        
        if not joint_reading:
            print 
            "self.gurdy_joint_dictionary="+str(self.gurdy_joint_dictionary)
            print 
            "joint_name===>"+str(joint_name)
            assert "There is no data about that joint"
        clean_joint_reading = self.convert_angle_to_unitary(angle=joint_reading)
        clean_value = self.convert_angle_to_unitary(angle=value)
        dif_angles = self.assertAlmostEqualAngles(clean_joint_reading, clean_value)
        similar = dif_angles <= error
        if not similar:
            rospy.logerr("The joint ="+str(joint_name)+", hasnt reached yet ="+str(value)+",difference="+str(dif_angles)+">"+str(error))
        return similar

    def gurdy_movement_look(self, upperlegM1_angle, upperlegM2_angle,
        upperlegM3_angle, lowerlegM1_value, lowerlegM2_value ,lowerlegM3_value,
        headarm_value, headforearm_value, upperlegM1_yaw_angle, upperlegM2_yaw_angle,
        upperlegM3_yaw_angle, headarm_yaw_value, basefoot_peg_M1_value,
        basefoot_peg_M2_value, basefoot_peg_M3_value):
        """
        Move:
        'head_upperlegM1_joint',
        'head_upperlegM2_joint',
        'head_upperlegM3_joint',
        'upperlegM1_lowerlegM1_joint',
        'upperlegM2_lowerlegM2_joint',
        'upperlegM3_lowerlegM3_joint'
        :return:
        """
        check_rate = 5.0
        position_upperlegM1_yaw = upperlegM1_yaw_angle
        position_upperlegM2_yaw = upperlegM2_yaw_angle
        position_upperlegM3_yaw = upperlegM3_yaw_angle
        position_upperlegM1 = upperlegM1_angle
        position_upperlegM2 = upperlegM2_angle
        position_upperlegM3 = upperlegM3_angle
        position_lowerlegM1 = lowerlegM1_value
        position_lowerlegM2 = lowerlegM2_value
        position_lowerlegM3 = lowerlegM3_value
        position_headarm_yaw = headarm_yaw_value
        position_headarm = headarm_value
        position_headforearm = headforearm_value
        position_basefoot_peg_M1_value = basefoot_peg_M1_value
        position_basefoot_peg_M2_value = basefoot_peg_M2_value
        position_basefoot_peg_M3_value = basefoot_peg_M3_value
        similar_upperlegM1_yaw = False
        similar_upperlegM2_yaw = False
        similar_upperlegM3_yaw = False
        similar_upperlegM1 = False
        similar_upperlegM2 = False
        similar_upperlegM3 = False
        similar_lowerlegM1 = False
        similar_lowerlegM2 = False
        similar_lowerlegM3 = False
        similar_headarm_yaw = False
        similar_headarm = False
        similar_headforearm = False
        similar_basefoot_peg_M1_value = False
        similar_basefoot_peg_M2_value = False
        similar_basefoot_peg_M3_value = False
        rate = rospy.Rate(check_rate)
        while not (similar_upperlegM1 and similar_upperlegM2 and
        similar_upperlegM3 and similar_lowerlegM1 and similar_lowerlegM2 and
        similar_lowerlegM3 and similar_headarm and similar_headforearm and
        similar_upperlegM1_yaw and similar_upperlegM2_yaw and similar_upperlegM3_yaw
        and similar_headarm_yaw and similar_basefoot_peg_M1_value and
        similar_basefoot_peg_M2_value and similar_basefoot_peg_M3_value):
            self.move_gurdy_all_joints(position_upperlegM1,
            position_upperlegM2,
            position_upperlegM3,
            position_lowerlegM1,
            position_lowerlegM2,
            position_lowerlegM3,
            position_headarm,
            position_headforearm,
            position_upperlegM1_yaw,
            position_upperlegM2_yaw,
            position_upperlegM3_yaw,
            position_headarm_yaw,
            position_basefoot_peg_M1_value,
            position_basefoot_peg_M2_value,
            position_basefoot_peg_M3_value)
            similar_upperlegM1 = self.gurdy_check_continuous_joint_value(joint_name="head_upperlegM1_joint", value=position_upperlegM1)
            similar_upperlegM2 = self.gurdy_check_continuous_joint_value(joint_name="head_upperlegM2_joint", value=position_upperlegM2)
            similar_upperlegM3 = self.gurdy_check_continuous_joint_value(joint_name="head_upperlegM3_joint", value=position_upperlegM3)
            similar_lowerlegM1 = self.gurdy_check_continuous_joint_value(joint_name="upperlegM1_lowerlegM1_joint", value=position_lowerlegM1)
            similar_lowerlegM2 = self.gurdy_check_continuous_joint_value(joint_name="upperlegM2_lowerlegM2_joint", value=position_lowerlegM2)
            similar_lowerlegM3 = self.gurdy_check_continuous_joint_value(joint_name="upperlegM3_lowerlegM3_joint", value=position_lowerlegM3)
            similar_headarm = self.gurdy_check_continuous_joint_value(joint_name="head_arm_joint",value=position_headarm)
            similar_headforearm = self.gurdy_check_continuous_joint_value(joint_name="head_forearm_joint", value=position_headforearm)
            similar_upperlegM1_yaw = self.gurdy_check_continuous_joint_value(joint_name="upperlegM1_yaw_joint", value=position_upperlegM1_yaw)
            similar_upperlegM2_yaw = self.gurdy_check_continuous_joint_value(joint_name="upperlegM2_yaw_joint", value=position_upperlegM2_yaw)
            similar_upperlegM3_yaw = self.gurdy_check_continuous_joint_value(joint_name="upperlegM3_yaw_joint", value=position_upperlegM3_yaw)
            similar_headarm_yaw = self.gurdy_check_continuous_joint_value(joint_name="head_arm_yaw_joint", value=position_headarm_yaw)
            similar_basefoot_peg_M1_value = self.gurdy_check_continuous_joint_value(joint_name="basefoot_peg_M1_basefoot_peg_M1_joint_joint",value=position_basefoot_peg_M1_value)
            similar_basefoot_peg_M2_value = self.gurdy_check_continuous_joint_value(joint_name="basefoot_peg_M2_basefoot_peg_M2_joint_joint",value=position_basefoot_peg_M2_value)
            similar_basefoot_peg_M3_value = self.gurdy_check_continuous_joint_value(joint_name="basefoot_peg_M3_basefoot_peg_M3_joint_joint",value=position_basefoot_peg_M3_value)
            rate.sleep()
    def gurdy_init_pos_sequence(self):
        """
        UPPER limits lower="-1.55" upper="0.0"
        LOWER limits lower="-2.9" upper="1.5708"
        :return:
        """
        rospy.loginfo("Start Init pos sequence...")
        upperlegM1_yaw_angle = 0.0
        upperlegM2_yaw_angle = 0.0
        upperlegM3_yaw_angle = 0.0
        upperlegM1_angle = -1.55
        upperlegM2_angle = -1.55
        upperlegM3_angle = -1.55
        lowerlegM1_angle = -1.55
        lowerlegM2_angle = -1.55
        lowerlegM3_angle = -1.55
        headarm_yaw_angle = 0.0
        headarm_angle = 0.0
        headforearm_angle = 0.0
        basefoot_peg_M1_value = 0.0
        basefoot_peg_M2_value = 0.0
        basefoot_peg_M3_value = 0.0
        self.gurdy_movement_look(upperlegM1_angle,
        upperlegM2_angle,
        upperlegM3_angle,
        lowerlegM1_angle,
        lowerlegM2_angle,
        lowerlegM3_angle,
        headarm_angle,
        headforearm_angle,
        upperlegM1_yaw_angle,
        upperlegM2_yaw_angle,
        upperlegM3_yaw_angle,
        headarm_yaw_angle,
        basefoot_peg_M1_value,
        basefoot_peg_M2_value,
        basefoot_peg_M3_value)
        rospy.loginfo("Start Init pos sequence... END")

    def gurdy_hop(self, num_hops=15):
        """
        UPPER limits lower="-1.55" upper="0.0"
        LOWER limits lower="-2.9" upper="1.5708"
        :return:
        """
        upper_delta = 1
        basic_angle = -1.55
        angle_change = random.uniform(0.2, 0.7)
        upperlegM_angle = basic_angle
        lowerlegM_angle = basic_angle - upper_delta * angle_change * 2.0
        headarm_angle = upperlegM_angle
        headforearm_angle = lowerlegM_angle
        upper_yaw_angle = 0.7
        upperleg_yaw_angle = upper_yaw_angle
        basefoot_max = 0.05
        basefoot_min = 0.0
        basefoot_peg_M1_value = basefoot_min
        basefoot_peg_M2_value = basefoot_min
        basefoot_peg_M3_value = basefoot_min
        #self.gurdy_init_pos_sequence()
        for repetitions in range(num_hops):
            self.gurdy_movement_look(upperlegM_angle,
            upperlegM_angle,
            upperlegM_angle,
            lowerlegM_angle,
            lowerlegM_angle,
            lowerlegM_angle,
            headarm_angle,
            headforearm_angle,
            upperleg_yaw_angle,
            upperleg_yaw_angle,
            upperleg_yaw_angle,
            upperleg_yaw_angle,
            basefoot_peg_M1_value,
            basefoot_peg_M2_value,
            basefoot_peg_M3_value)
            upper_delta *= -1
            if upper_delta < 0:
                upperlegM_angle = basic_angle + angle_change
                upperleg_yaw_angle = upper_yaw_angle
                basefoot_peg_M1_value = basefoot_min
                basefoot_peg_M2_value = basefoot_min
                basefoot_peg_M3_value = basefoot_min
            else:
                upperlegM_angle = basic_angle
                upperleg_yaw_angle = -1* upper_yaw_angle
                basefoot_peg_M1_value = basefoot_max
                basefoot_peg_M2_value = basefoot_max
                basefoot_peg_M3_value = basefoot_max
                lowerlegM_angle = basic_angle - upper_delta * angle_change * 2.0
                headarm_angle = upperlegM_angle
                headforearm_angle = lowerlegM_angle
    def jump(self, min_val=0.0, max_val=0.05, time_jump=1.0,direction="forwards"):
        """
        It lowers the primsatic joints and then thrusts to maximum.
        """
        rospy.logwarn("time_jump=="+str(time_jump))
        if direction=="forwards":
            if time_jump != 0.0:
                rospy.logwarn("PREpare....")
                self.gurdy_move_ony_primsatic(foot_m1=min_val,foot_m2=min_val,foot_m3=min_val)
                time.sleep(time_jump)
                rospy.logwarn("JUMP!")

                self.gurdy_move_ony_primsatic(foot_m1=max_val,foot_m2=min_val,foot_m3=min_val)
                time.sleep(time_jump)

        elif direction=="backwards":
            if time_jump != 0.0:
                rospy.logwarn("PREpare....")
                self.gurdy_move_ony_primsatic(foot_m1=min_val,foot_m2=min_val,foot_m3=min_val)
                time.sleep(time_jump)
                rospy.logwarn("JUMP!")
        
                self.gurdy_move_ony_primsatic(foot_m1=min_val,foot_m2=max_val,foot_m3=max_val)
                time.sleep(time_jump)

        else:
            if time_jump != 0.0:
                rospy.logwarn("PREpare....")
                self.gurdy_move_ony_primsatic(foot_m1=min_val,foot_m2=min_val,foot_m3=min_val)
                time.sleep(time_jump)
                rospy.logwarn("JUMP!")
                self.gurdy_move_ony_primsatic(foot_m1=max_val,foot_m2=max_val,foot_m3=max_val)
                time.sleep(time_jump)
    def move_with_cmd_vel(self):
        """
        We read the /cmd_vel topic and based on that we move gurdy
        """
        pass

    def init_move_cmd_vel_pose(self):
        upperlegM1_angle = -1.55
        upperlegM2_angle = -1.55
        upperlegM3_angle = -1.55
        lowerlegM1_angle = -0.7
        lowerlegM2_angle = -1.55
        lowerlegM3_angle = -1.55
        headarm_angle = 0.0
        headforearm_angle = 0.0
        upperlegM1_yaw_angle = 0.0
        upperlegM2_yaw_angle = 0.7
        upperlegM3_yaw_angle = -0.7
        headarm_yaw_angle = 0.0
        basefoot_peg_M1_value = 0.0
        basefoot_peg_M2_value = 0.0
        basefoot_peg_M3_value = 0.0
        self.gurdy_movement_look(upperlegM1_angle=upperlegM1_angle,
        upperlegM2_angle=upperlegM2_angle,
        upperlegM3_angle=upperlegM3_angle,
        lowerlegM1_value=lowerlegM1_angle,
        lowerlegM2_value=lowerlegM2_angle,
        lowerlegM3_value=lowerlegM3_angle,
        headarm_value=headarm_angle,
        headforearm_value=headforearm_angle,
        upperlegM1_yaw_angle=upperlegM1_yaw_angle,
        upperlegM2_yaw_angle=upperlegM2_yaw_angle,
        upperlegM3_yaw_angle=upperlegM3_yaw_angle,
        headarm_yaw_value=headarm_yaw_angle,
        basefoot_peg_M1_value=basefoot_peg_M1_value,
        basefoot_peg_M2_value=basefoot_peg_M2_value,
        basefoot_peg_M3_value=basefoot_peg_M3_value)

    def _cmd_vel_callback(self, msg):
        self.twist_value = msg

    def change_move_direction(self, direction):
        """
        It will change the angle of M1 leg to move in different directions
        """
        if direction == "fowards":
            angle = 0.0
        elif direction == "left":
            angle = -0.3
        elif direction == "right":
            angle = 0.3
        else:
            angle = 0.0
        self.gurdy_move_ony_m1_yaw(angle=angle)
        
    def decide_movement(self):
        angular_val = self.twist_value.angular.z
        direction = None
        
        if angular_val > 0.0:
            direction = "left"
        if angular_val < 0.0:
            direction = "right"
            self.change_move_direction(direction)
            jump_magnitude = self.twist_value.linear.x
        if jump_magnitude >= 0.0:
            direction = "forwards"
        else:
            direction = "backwards"
        if jump_magnitude == 0.0:
            jump_value_fixed_inv = 0.0
        else:
            max_value = 100.0
            min_value = 0.01
            jump_value_fixed = self.clamp(num=abs(jump_magnitude),min_value=min_value,max_value=max_value)
            jump_value_fixed_inv = 1.0 / jump_value_fixed
        return jump_value_fixed_inv, direction

    def clamp(self, num, min_value, max_value):
        return max(min(num, max_value), min_value)

    def movement_cmd_vel(self):
        """
        We use the cmd_vel topic to move Gurdy
        """
        self.init_move_cmd_vel_pose()
        while not rospy.is_shutdown():
            #self.gurdy_hop()
            jump_value_fixed, direction = self.decide_movement()
            self.jump(min_val=0.0, max_val=0.01, time_jump=jump_value_fixed,direction=direction)

    def movement_random_loop(self):
        """
        Executed movements in a random way
        :return:
        """
        rospy.loginfo("Start Moving Gurdy...")
        #self.gurdy_init_pos_sequence()
        self.init_move_cmd_vel_pose()
        while not rospy.is_shutdown():
        #self.gurdy_hop()
           self.jump(min_val=0.0, max_val=0.01, time_jump=0.1)

    def gurdy_flip(self):
        """
        UPPER limits lower="-1.55" upper="0.0"
        LOWER limits lower="-2.9" upper="1.5708"
        :return:
        """
        upperlegM1_angle = -0.7
        upperlegM2_angle = -0.7
        upperlegM3_angle = 0.0
        lowerlegM1_angle = 0.0
        lowerlegM2_angle = 0.0
        lowerlegM3_angle = 0.7
        headarm_angle = 1.5
        headforearm_angle = 2.7
        upperlegM1_yaw_angle = 0.0
        upperlegM2_yaw_angle = 0.0
        upperlegM3_yaw_angle = 0.0
        headarm_yaw_angle = 0.0
        basefoot_peg_M1_value = 0.0
        basefoot_peg_M2_value = 0.0
        basefoot_peg_M3_value = 0.0
        self.gurdy_movement_look(upperlegM1_angle=upperlegM1_angle,
        upperlegM2_angle=upperlegM2_angle,
        upperlegM3_angle=upperlegM3_angle,
        lowerlegM1_value=lowerlegM1_angle,
        lowerlegM2_value=lowerlegM2_angle,
        lowerlegM3_value=lowerlegM3_angle,
        headarm_value=headarm_angle,
        headforearm_value=headforearm_angle,
        upperlegM1_yaw_angle=upperlegM1_yaw_angle,
        upperlegM2_yaw_angle=upperlegM2_yaw_angle,
        upperlegM3_yaw_angle=upperlegM3_yaw_angle,
        headarm_yaw_value=headarm_yaw_angle,
        basefoot_peg_M1_value=basefoot_peg_M1_value,
        basefoot_peg_M2_value=basefoot_peg_M2_value,
        basefoot_peg_M3_value=basefoot_peg_M3_value)
        upperlegM1_angle = -1.55
        upperlegM2_angle = -1.55
        upperlegM3_angle = -1.55
        lowerlegM1_angle = -2.9
        lowerlegM2_angle = -2.9
        lowerlegM3_angle = -2.9
        self.gurdy_movement_look(upperlegM1_angle=upperlegM1_angle,
        upperlegM2_angle=upperlegM2_angle,
        upperlegM3_angle=upperlegM3_angle,
        lowerlegM1_value=lowerlegM1_angle,
        lowerlegM2_value=lowerlegM2_angle,
        lowerlegM3_value=lowerlegM3_angle,
        headarm_value=headarm_angle,
        headforearm_value=headforearm_angle,
        upperlegM1_yaw_angle=upperlegM1_yaw_angle,
        upperlegM2_yaw_angle=upperlegM2_yaw_angle,
        upperlegM3_yaw_angle=upperlegM3_yaw_angle,
        headarm_yaw_value=headarm_yaw_angle,
        basefoot_peg_M1_value=basefoot_peg_M1_value,
        basefoot_peg_M2_value=basefoot_peg_M2_value,
        basefoot_peg_M3_value=basefoot_peg_M3_value)

    def gurdy_basic_stance(self):
        """
        UPPER limits lower="-1.55" upper="0.0"
        LOWER limits lower="-2.9" upper="1.5708"
        :return:
        """
        upperlegM1_angle = -1.55
        upperlegM2_angle = -1.55
        upperlegM3_angle = -1.55
        lowerlegM1_angle = -2.9
        lowerlegM2_angle = -2.9
        lowerlegM3_angle = -2.9
        headarm_angle = 1.5
        headforearm_angle = 2.7
        upperlegM1_yaw_angle = 0.0
        upperlegM2_yaw_angle = 0.0
        upperlegM3_yaw_angle = 0.0
        headarm_yaw_angle = 0.0
        basefoot_peg_M1_value = 0.0
        basefoot_peg_M2_value = 0.0
        basefoot_peg_M3_value = 0.0
        self.gurdy_movement_look(upperlegM1_angle=upperlegM1_angle,
        upperlegM2_angle=upperlegM2_angle,
        upperlegM3_angle=upperlegM3_angle,
        lowerlegM1_value=lowerlegM1_angle,
        lowerlegM2_value=lowerlegM2_angle,
        lowerlegM3_value=lowerlegM3_angle,
        headarm_value=headarm_angle,
        headforearm_value=headforearm_angle,
        upperlegM1_yaw_angle=upperlegM1_yaw_angle,
        upperlegM2_yaw_angle=upperlegM2_yaw_angle,
        upperlegM3_yaw_angle=upperlegM3_yaw_angle,
        headarm_yaw_value=headarm_yaw_angle,
        basefoot_peg_M1_value=basefoot_peg_M1_value,
        basefoot_peg_M2_value=basefoot_peg_M2_value,
        basefoot_peg_M3_value=basefoot_peg_M3_value)

    def execute_movement_gurdy(self, movement):
        """
        Executes a movement given
        """
        if movement == "flip":
            self.gurdy_flip()
        elif movement == "basic_stance":
            self.gurdy_basic_stance()
        elif movement == "movement_stance":
            self.init_move_cmd_vel_pose()
        elif movement == "init_stance":
            self.gurdy_init_pos_sequence()
        elif movement == "jump":
            self.gurdy_init_pos_sequence()
            self.jump(min_val=0.0, max_val=0.1, time_jump=1.0, direction="in_place")
        else:
            rospy.logerr("Movement "+str(movement)+" not supported yet.")

if __name__ == "__main__":
    rospy.init_node('jointmover_demo', anonymous=True)
    gurdy_jointmover_object = gurdyJointMover()
    gurdy_jointmover_object.movement_cmd_vel()
