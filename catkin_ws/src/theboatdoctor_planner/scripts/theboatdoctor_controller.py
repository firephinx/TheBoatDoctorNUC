#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Empty, String, Bool
from geometry_msgs.msg import Pose2D, Twist
from sensor_msgs.msg import JointState, Range, Imu

class TheBoatDoctorController:
    def __init__(self):
        rospy.init_node('TheBoatDoctorController')
        self.current_turntable_theta = 0.0
        self.current_base_position = [0.0,0.0,0.0]
        self.current_gantry_position = [0.0,0.0]
        self.current_arm_joint_position = [0.0,0.0,0.0]
        self.front_ultrasonic_range = 0.0
        self.right_ultrasonic_range = 0.0

        self.minimum_robot_base_x_threshold = 0.23
        self.minimum_robot_base_y_threshold = 0.23
        self.maximum_robot_base_x_threshold = 1.0
        self.maximum_robot_base_y_threshold = 1.5
        self.minimum_x_gantry_threshold = 0.0
        self.maximum_x_gantry_threshold = 0.23
        self.minimum_z_gantry_threshold = 0.0
        self.maximum_z_gantry_threshold = 0.333
        self.minimum_turntable_threshold = -math.pi / 2
        self.maximum_turntable_threshold = math.pi * 2 / 3 

        self.reset_pub = rospy.Publisher('/TheBoatDoctor/Reset', Empty, queue_size=1)
        self.home_pub = rospy.Publisher('/TheBoatDoctor/Home', Empty, queue_size=1)
        self.home_gantry_pub = rospy.Publisher('/TheBoatDoctor/HomeGantry', Empty, queue_size=1)
        self.home_turntable_pub = rospy.Publisher('/TheBoatDoctor/HomeTurntable', Empty, queue_size=1)
        self.move_robot_base_pub = rospy.Publisher('/TheBoatDoctor/move_robot_base', Pose2D, queue_size=10);
        self.move_gantry_pub = rospy.Publisher('/TheBoatDoctor/move_gantry', Pose2D, queue_size=10);
        self.turn_turntable_pub = rospy.Publisher('/TheBoatDoctor/turn_turntable', Pose2D, queue_size=10);
        self.move_arm_pub = rospy.Publisher('/TheBoatDoctor/move_arm', JointState, queue_size=10)
        self.pump_pub = rospy.Publisher('/TheBoatDoctor/Pump_Switch', Bool, queue_size=1)
        self.led_pub = rospy.Publisher('/TheBoatDoctor/LED_Switch', Bool, queue_size=1)

        rospy.sleep(1)

    def front_ultrasonic_range_callback(self, range_msg):
        self.front_ultrasonic_range = range_msg.range

    def right_ultrasonic_range_callback(self, range_msg):
        self.right_ultrasonic_range = range_msg.range

    def ultrasonic_pose_callback(self, pose_2d_msg):
        self.current_base_position[0] = pose_2d_msg.x
        self.current_base_position[1] = pose_2d_msg.y
        self.current_base_position[2] = pose_2d_msg.theta

    def reset_robot(self):
        empty_msg = Empty()

        self.reset_pub.publish(empty_msg)
        self.reset_pub.publish(empty_msg)

    def home_arm(self):
        return self.move_arm([-math.pi / 2, -math.pi / 2, 0])

    def home_arm_with_goal_end_effector_angle(self, desired_end_effector_theta):
        return self.move_arm([-math.pi / 2, -math.pi / 2, desired_end_effector_theta])

    def position_arm_for_kinect_vision(self):
        return self.move_arm([-math.pi * 3 / 5, -math.pi * 3 / 5, 0])

    def home_gantry(self):

        empty_msg = Empty()

        self.home_gantry_pub.publish(empty_msg)
        self.home_gantry_pub.publish(empty_msg)

        try:
            done_homing_msg = rospy.wait_for_message('/TheBoatDoctor/done_homing', Bool, timeout = 10)
            return done_homing_msg.data and done_homing_arm
        except:
            print("Timeout waiting for done_homing_msg.")
            return False

    def home_turntable(self):

        empty_msg = Empty()

        self.home_turntable_pub.publish(empty_msg)
        self.home_turntable_pub.publish(empty_msg)

        try:
            done_homing_msg = rospy.wait_for_message('/TheBoatDoctor/done_homing', Bool, timeout = 5)
            return done_homing_msg.data and done_homing_arm
        except:
            print("Timeout waiting for done_homing_msg.")
            return False

    def home_robot(self):

        done_homing_arm = self.home_arm()

        empty_msg = Empty()

        self.home_pub.publish(empty_msg)
        self.home_pub.publish(empty_msg)

        try:
            done_homing_msg = rospy.wait_for_message('/TheBoatDoctor/done_homing', Bool, timeout = 10)
            return done_homing_msg.data and done_homing_arm
        except:
            print("Timeout waiting for done_homing_msg.")
            return False

    def move_robot_base(self, desired_robot_base_position):
        if(desired_robot_base_position[0] < self.minimum_robot_base_x_threshold or desired_robot_base_position[0] > self.maximum_robot_base_x_threshold):
            print("Desired Robot Base X Position (" + str(desired_robot_base_position[0]) + ") is outside of the possible X Robot Position range.")
            return False

        if(desired_robot_base_position[1] < self.minimum_robot_base_y_threshold or desired_robot_base_position[1] > self.maximum_robot_base_y_threshold):
            print("Desired Robot Base Y Position (" + str(desired_robot_base_position[1]) + ") is outside of the possible Z Robot Position range.")
            return False

        pose_2d_msg = Pose2D()
        pose_2d_msg.x = round(desired_robot_base_position[0], 4)
        pose_2d_msg.y = round(desired_robot_base_position[1], 4)
        pose_2d_msg.theta = desired_robot_base_position[2]

        self.move_robot_base_pub.publish(pose_2d_msg)
        self.move_robot_base_pub.publish(pose_2d_msg)

        try:
            done_moving_robot_base_msg = rospy.wait_for_message('/TheBoatDoctor/done_moving_robot_base', Bool, timeout = 30)
            return done_moving_robot_base_msg.data
        except:
            print("Timeout waiting for done_moving_robot_base_msg.")
            return False

    def move_gantry(self, desired_gantry_position):

        if(desired_gantry_position[0] < self.minimum_x_gantry_threshold or desired_gantry_position[0] > self.maximum_x_gantry_threshold):
            print("Desired X Gantry Position (" + str(desired_gantry_position[0]) + ") is outside of the possible X Gantry range.")
            return False

        if(desired_gantry_position[1] < self.minimum_z_gantry_threshold or desired_gantry_position[1] > self.maximum_z_gantry_threshold):
            print("Desired Z Gantry Position (" + str(desired_gantry_position[1]) + ") is outside of the possible Z Gantry range.")
            return False

        pose_2d_msg = Pose2D()
        pose_2d_msg.x = round(desired_gantry_position[0], 4)
        pose_2d_msg.y = round(desired_gantry_position[1], 4)

        self.move_gantry_pub.publish(pose_2d_msg)
        self.move_gantry_pub.publish(pose_2d_msg)

        try:
            done_moving_gantry_msg = rospy.wait_for_message('/TheBoatDoctor/done_moving_gantry', Bool, timeout = 10)
            return done_moving_gantry_msg.data
        except:
            print("Timeout waiting for done_moving_gantry_msg.")
            return False
        
    def turn_turntable(self, desired_theta):
        if(desired_theta < self.minimum_turntable_threshold or desired_theta > self.maximum_turntable_threshold):
            print("Desired Theta (" + str(desired_theta) + ") is outside of the possible turntable range.")
            return False

        pose_2d_msg = Pose2D()
        pose_2d_msg.theta = round(desired_theta, 4)

        self.turn_turntable_pub.publish(pose_2d_msg)
        self.turn_turntable_pub.publish(pose_2d_msg)

        try:
            done_turning_turntable_msg = rospy.wait_for_message('/TheBoatDoctor/done_turning_turntable', Bool, timeout = 5)
            return done_turning_turntable_msg.data
        except:
            print("Timeout waiting for done_turning_turntable_msg.")
            return False

    def move_arm(self, desired_arm_angles):
        joint_state_msg = JointState()
        joint_state_msg.name = ['elbow','wrist','end effector']
        joint_state_msg.position = [desired_arm_angles[0], desired_arm_angles[1], desired_arm_angles[2]]

        self.move_arm_pub.publish(joint_state_msg)
        self.move_arm_pub.publish(joint_state_msg)

        try:
            done_moving_arm_msg = rospy.wait_for_message('TheBoatDoctor/done_moving_arm', Bool, timeout = 5)
            return done_moving_arm_msg.data
        except:
            print("Timeout waiting for done_moving_arm_msg.")
            return False

    def get_current_base_position(self):
        try:
            pose_2d_msg = rospy.wait_for_message('/TheBoatDoctor/ultrasonic_pose', Pose2D, timeout = 1)
            current_base_position = [pose_2d_msg.x,pose_2d_msg.y,pose_2d_msg.theta]
            return current_base_position
        except:
            print("Timeout waiting for pose_2d_msg.")
            return [0.0, 0.0, 0.0]

    def get_current_turntable_degree(self):
        try:
            joint_state_msg = rospy.wait_for_message('/TheBoatDoctor/joint_states', JointState, timeout = 1)
            turntable_ind = 0
            for i in xrange(len(joint_state_msg.name)):
                if(joint_state_msg.name[i] == 'turntable'):
                    turntable_ind = i
                    break
                else:
                    i = i + 1
            return math.degrees(joint_state_msg.position[turntable_ind])
        except:
            print("Timeout waiting for joint_state_msg.")
            return 0.0

    def get_current_gantry_position(self):
        try:
            joint_state_msg = rospy.wait_for_message('/TheBoatDoctor/joint_states', JointState, timeout = 1)
            x_gantry_ind = 0
            z_gantry_ind = 0
            for i in xrange(len(joint_state_msg.name)):
                if(joint_state_msg.name[i] == 'X_motion'):
                    x_gantry_ind = i
                elif(joint_state_msg.name[i] == 'Z_motion'):
                    z_gantry_ind = i
                i = i + 1
            return [joint_state_msg.position[x_gantry_ind], joint_state_msg.position[z_gantry_ind]]
        except:
            print("Timeout waiting for joint_state_msg.")
            return [0.0, 0.0]

    def pump_switch(self, switch):
        bool_msg = Bool()
        if(switch == "on"):
            bool_msg.data = True
        else:
            bool_msg.data = False
        self.pump_pub.publish(bool_msg)
        self.pump_pub.publish(bool_msg)
        try:
            pump_status_msg = rospy.wait_for_message('/TheBoatDoctor/pump_status', Bool, timeout = 1)
            if(pump_status_msg.data == bool_msg.data):
                return True
            else:
                return False
        except:
            print("Timeout waiting for pump_status_msg.")
            return False

    def led_switch(self, switch):
        bool_msg = Bool()
        if(switch == "on"):
            bool_msg.data = True
        else:
            bool_msg.data = False

        self.led_pub.publish(bool_msg)
        self.led_pub.publish(bool_msg)

        try:
            led_status_msg = rospy.wait_for_message('/TheBoatDoctor/led_status', Bool, timeout = 1)
            if(led_status_msg.data == bool_msg.data):
                return True
            else:
                return False
        except:
            print("Timeout waiting for led_status_msg.")
            return False