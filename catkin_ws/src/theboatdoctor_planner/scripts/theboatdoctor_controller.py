#!/usr/bin/env python

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

		self.home_pub = rospy.Publisher('/TheBoatDoctor/Home', Empty, queue_size=1)
		self.move_robot_base_pub = rospy.Publisher('/TheBoatDoctor/move_robot_base', Pose2D, queue_size=10);
		self.move_gantry_pub = rospy.Publisher('/TheBoatDoctor/move_gantry', Pose2D, queue_size=10);
		self.turn_turntable_pub = rospy.Publisher('/TheBoatDoctor/turn_turntable', Pose2D, queue_size=10);
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

	def home_robot(self):
		empty_msg = Empty()
		self.home_pub.publish(empty_msg)
		self.home_pub.publish(empty_msg)
		done_homing_msg = rospy.wait_for_message('/TheBoatDoctor/done_homing', Bool)
		return done_homing_msg.data

	def move_robot_base(self, desired_robot_base_position):
		pose_2d_msg = Pose2D()
		pose_2d_msg.x = desired_robot_base_position[0]
		pose_2d_msg.y = desired_robot_base_position[1]
		pose_2d_msg.theta = desired_robot_base_position[2]
		self.move_robot_base_pub.publish(pose_2d_msg)
		self.move_robot_base_pub.publish(pose_2d_msg)
		done_moving_robot_base_msg = rospy.wait_for_message('/TheBoatDoctor/done_moving_robot_base', Bool)
		return done_moving_robot_base_msg.data

	def move_gantry(self, desired_gantry_position):
		pose_2d_msg = Pose2D()
		pose_2d_msg.x = desired_gantry_position[0]
		pose_2d_msg.y = desired_gantry_position[1]
		self.move_gantry_pub.publish(pose_2d_msg)
		self.move_gantry_pub.publish(pose_2d_msg)
		done_moving_gantry_msg = rospy.wait_for_message('/TheBoatDoctor/done_moving_gantry', Bool)
		return done_moving_gantry_msg.data
		
	def turn_turntable(self, desired_theta):
		pose_2d_msg = Pose2D()
		pose_2d_msg.theta = desired_theta
		self.turn_turntable_pub.publish(pose_2d_msg)
		self.turn_turntable_pub.publish(pose_2d_msg)
		done_turning_turntable_msg = rospy.wait_for_message('/TheBoatDoctor/done_turning_turntable', Bool)
		return done_turning_turntable_msg.data

	def get_current_position(self):
		pose_2d_msg = rospy.wait_for_message('/TheBoatDoctor/ultrasonic_pose', Pose2D)
		current_position = [pose_2d_msg.x,pose_2d_msg.y,pose_2d_msg.theta]
		return current_position

	def get_current_turntable_position(self):
		joint_state_msg = rospy.wait_for_message('/TheBoatDoctor/joint_states', JointState)
		turntable_ind = 0
		for i in xrange(len(joint_state_msg.name)):
			if(joint_state_msg.name[i] == 'turntable'):
				turntable_ind = i
				break
			else:
				i = i + 1
		return joint_state_msg.position[turntable_ind]

	def get_current_gantry_position(self):
		joint_state_msg = rospy.wait_for_message('/TheBoatDoctor/joint_states', JointState)
		x_gantry_ind = 0
		z_gantry_ind = 0
		for i in xrange(len(joint_state_msg.name)):
			if(joint_state_msg.name[i] == 'X_motion'):
				x_gantry_ind = i
			elif(joint_state_msg.name[i] == 'Z_motion'):
				z_gantry_ind = i
			i = i + 1
		return [joint_state_msg.position[x_gantry_ind], joint_state_msg.position[z_gantry_ind]]

	def pump_switch(self, switch):
		bool_msg = Bool()
		bool_msg.data = switch
		self.pump_pub.publish(bool_msg)
		self.pump_pub.publish(bool_msg)
		pump_status_msg = rospy.wait_for_message('/TheBoatDoctor/pump_status', Bool)
		return pump_status_msg.data

	def led_switch(self, switch):
		bool_msg = Bool()
		bool_msg.data = switch
		self.led.publish(bool_msg)
		self.led.publish(bool_msg)
		led_status_msg = rospy.wait_for_message('/TheBoatDoctor/led_status', Bool)
		return led_status_msg.data