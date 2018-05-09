#!/usr/bin/env python

import time
import numpy
import math
from theboatdoctor_controller import TheBoatDoctorController
from theboatdoctor_ik import TheBoatDoctorIK
from theboatdoctor_cv import TheBoatDoctorCV

#All measurements in meters and radians

class TheBoatDoctorPlanner:
    def __init__(self):
        ## Tuning Parameters

        ## Vertical Station Offsets
        ## V1 Offsets
        self.raspberry_pi_camera_vertical_station_v1_x_offset = -0.2
        self.raspberry_pi_camera_vertical_station_v1_z_offset = -0.06
        self.vertical_station_v1_goal_x_offset = -0.08
        self.vertical_station_v1_goal_z_offset = 0.01

        ## V2 Offsets
        self.raspberry_pi_camera_vertical_station_v2_x_offset = -0.22
        self.raspberry_pi_camera_vertical_station_v2_z_offset = -0.06
        self.vertical_station_v2_goal_x_offset = -0.1
        self.vertical_station_v2_goal_z_offset = 0.02

        ## V3 Offsets
        self.raspberry_pi_camera_vertical_station_v3_x_offset = -0.2
        self.raspberry_pi_camera_vertical_station_v3_z_offset = -0.07
        self.vertical_station_v3_goal_x_offset = -0.07
        self.vertical_station_v3_goal_z_offset = 0.0

        ## Breaker Offsets
        self.raspberry_pi_camera_vertical_station_breaker_x_offset = -0.2
        self.raspberry_pi_camera_vertical_station_breaker_z_offset = -0.06
        self.breaker_b_goal_wrist_flip_up_theta = math.pi / 9
        self.breaker_a_goal_wrist_flip_down_theta = -math.pi / 18
        self.breaker_b_goal_wrist_flip_down_theta = -math.pi / 4

        self.breaker_1_turntable_degree_offset = -5
        self.breaker_2_turntable_degree_offset = 0.0
        self.breaker_3_turntable_degree_offset = 5

        self.breaker_1_a_x_offset = -0.05
        self.breaker_1_a_z_offset = 0.01
        self.breaker_2_a_x_offset = -0.03
        self.breaker_2_a_z_offset = 0.06
        self.breaker_3_a_x_offset = -0.01
        self.breaker_3_a_z_offset = 0.14

        self.breaker_1_b_x_offset = -0.05
        self.breaker_1_b_z_offset = 0.01
        self.breaker_2_b_x_offset = -0.03
        self.breaker_2_b_z_offset = 0.06
        self.breaker_3_b_x_offset = -0.01
        self.breaker_3_b_z_offset = 0.1

        self.corner_e_breaker_1_a_turntable_degree_offset = -7
        self.corner_e_breaker_2_a_turntable_degree_offset = -3
        self.corner_e_breaker_3_a_turntable_degree_offset = 3

        self.corner_e_breaker_1_a_x_offset = -0.08
        self.corner_e_breaker_1_a_z_offset = -0.07
        self.corner_e_breaker_2_a_x_offset = -0.075
        self.corner_e_breaker_2_a_z_offset = -0.01
        self.corner_e_breaker_3_a_x_offset = -0.01
        self.corner_e_breaker_3_a_z_offset = 0.07

        self.corner_e_breaker_1_b_turntable_degree_offset = -10
        self.corner_e_breaker_2_b_turntable_degree_offset = -3
        self.corner_e_breaker_3_b_turntable_degree_offset = 2

        self.corner_e_breaker_1_b_x_offset = -0.088
        self.corner_e_breaker_1_b_z_offset = -0.1
        self.corner_e_breaker_2_b_x_offset = -0.075
        self.corner_e_breaker_2_b_z_offset = -0.03
        self.corner_e_breaker_3_b_x_offset = -0.044
        self.corner_e_breaker_3_b_z_offset = 0.03

        self.corner_f_breaker_1_a_turntable_degree_offset = -2
        self.corner_f_breaker_2_a_turntable_degree_offset = 2.5
        self.corner_f_breaker_3_a_turntable_degree_offset = 7

        self.corner_f_breaker_1_a_x_offset = -0.04
        self.corner_f_breaker_1_a_z_offset = 0.03
        self.corner_f_breaker_2_a_x_offset = -0.03
        self.corner_f_breaker_2_a_z_offset = 0.09
        self.corner_f_breaker_3_a_x_offset = -0.01
        self.corner_f_breaker_3_a_z_offset = 0.18

        self.corner_f_breaker_1_b_turntable_degree_offset = -2.5
        self.corner_f_breaker_2_b_turntable_degree_offset = 2.5
        self.corner_f_breaker_3_b_turntable_degree_offset = 6

        self.corner_f_breaker_1_b_x_offset = -0.045
        self.corner_f_breaker_1_b_z_offset = 0.01
        self.corner_f_breaker_2_b_x_offset = -0.04
        self.corner_f_breaker_2_b_z_offset = 0.04
        self.corner_f_breaker_3_b_x_offset = -0.02
        self.corner_f_breaker_3_b_z_offset = 0.11

        ## Horizontal Station Offsets
        ## V1 Offsets
        self.raspberry_pi_camera_horizontal_station_v1_x_offset = -0.16
        self.raspberry_pi_camera_horizontal_station_v1_z_offset = -0.02
        self.horizontal_station_v1_goal_x_offset = -0.09
        self.horizontal_station_v1_goal_z_offset = -0.06

        ## V3 Offsets
        self.corner_e_raspberry_pi_camera_horizontal_station_v3_x_offset = -0.17
        self.corner_e_raspberry_pi_camera_horizontal_station_v3_z_offset = 0.03
        self.corner_e_horizontal_station_open_v3_goal_x_offset = 0.08
        self.corner_e_horizontal_station_open_v3_goal_z_offset = -0.07
        self.corner_e_horizontal_station_open_v3_goal_x_gantry_offset = 0.05
        self.corner_e_horizontal_station_closed_v3_goal_x_offset = 0.13
        self.corner_e_horizontal_station_closed_v3_goal_z_offset = -0.04
        self.corner_e_horizontal_station_closed_v3_goal_x_gantry_offset = 0.15
        self.corner_e_horizontal_station_closed_v3_goal_2_x_offset = -0.03
        self.corner_e_horizontal_station_closed_v3_goal_2_z_offset = 0.08
        self.corner_e_horizontal_station_closed_v3_wrist_flip_theta = -math.pi / 2

        self.horizontal_station_v3_degree_offset = 5

        self.raspberry_pi_camera_horizontal_station_v3_x_offset = -0.17
        self.raspberry_pi_camera_horizontal_station_v3_z_offset = 0.03
        self.horizontal_station_open_v3_goal_x_offset = 0.08
        self.horizontal_station_open_v3_goal_z_offset = -0.04
        self.horizontal_station_open_v3_goal_x_gantry_offset = 0.05
        self.horizontal_station_closed_v3_goal_x_offset = 0.13
        self.horizontal_station_closed_v3_goal_z_offset = -0.04
        self.horizontal_station_closed_v3_goal_x_gantry_offset = 0.15
        self.horizontal_station_closed_v3_goal_2_x_offset = -0.03
        self.horizontal_station_closed_v3_goal_2_z_offset = 0.08
        self.horizontal_station_closed_v3_wrist_flip_theta = -math.pi / 2

        ## Thresholds
        self.angle_threshold_in_degrees = 3
        self.task_completion_angle_threshold_in_degrees = 15
        self.v3_valve_threshold = 15

        self.tbd_controller = TheBoatDoctorController()
        self.tbd_ik = TheBoatDoctorIK()

        self.done_homing_flag = False
        self.done_moving_robot_base_flag = False
        self.done_moving_gantry_flag = False
        self.done_moving_arm_flag = False

        self.current_base_position = self.tbd_controller.get_current_base_position()
        self.desired_station_angle_in_degrees = 0
        self.desired_z_movement_distance = 0
        self.station_orientation = ''
        self.breaker_positions_in_3d_robot_coordinates = [0, 0, 0]
        self.joint_angles_for_goal_positions = numpy.zeros([1,6])

    def cam_to_ik(self, cam_coord):
        r_y_90 = numpy.array([ [0, 0, 1], [0, 1, 0], [-1, 0, 0] ])
        return r_y_90.dot(cam_coord)

    def determine_station_base_coords(self):
        print("Station: " + self.station)
        if (self.station == "A"):
            self.station_base_coords = [0.419, 1.264, 0.0]
        elif (self.station == "B"):
            self.station_base_coords = [0.419, 0.984, 0.0]
        elif (self.station == "C"):
            #self.station_base_coords = [0.419, 0.734, 0.0]
            self.station_base_coords = [0.389, 0.734, 0.0]
        elif (self.station == "D"):
            self.station_base_coords = [0.419, 0.464, 0.0]
        elif (self.station == "E"):
            self.station_base_coords = [0.339, 0.314, 0.0]
        elif (self.station == "F"):
            self.station_base_coords = [0.239, 0.414, 0.0]
        elif (self.station == "G"):
            self.station_base_coords = [0.389, 0.414, 0.0]
        elif (self.station == "H"):
            self.station_base_coords = [0.669, 0.369, 0.0]
        else :
            self.station_base_coords = [0.419, 0.419, 0.0]

        print("Desired Base Position: [" + str(self.station_base_coords[0]) + ", " + str(self.station_base_coords[1]) + ", " + str(self.station_base_coords[2]) + "]")

    def determine_station_turntable_degree(self):
        print("Station: " + self.station)
        if(self.station == "A" or self.station == "B" or self.station == "C" or self.station == "D"):
            #ensure the turntable is at 0 degrees
            self.station_turntable_degree = 0
        elif(self.station == "G" or self.station == "H"):
            # Ensure the turntable is at 90 degrees
            self.station_turntable_degree = 90
        elif(self.station == "F"):
            # ensure the turntable is at 60 degrees
            self.station_turntable_degree = 75
        elif (self.station == "E"):
            #ensure the turntable is at 13 degrees
            self.station_turntable_degree = 15
        else:
            #ensure the turntable is at 0 degrees
            self.station_turntable_degree = 0

        print("Desired Turntable Degree: [" + str(self.station_turntable_degree) + "]")

    def reset_robot(self):
        print("Resetting robot")
        self.tbd_controller.reset_robot()

    def home_robot(self):
        print("Homing robot")
        self.done_homing_flag = self.tbd_controller.home_robot()
        while(self.done_homing_flag != True):
            self.done_homing_flag = self.tbd_controller.home_robot()
        print("Done homing robot")

    def home_arm(self):
        print("Homing arm")
        self.tbd_controller.home_arm()
        print("Done homing arm")

    def home_arm_with_goal_end_effector_angle(self):
        print("Homing arm with goal end effector angle")
        self.tbd_controller.home_arm_with_goal_end_effector_angle(self.desired_end_effector_angle_in_radians)
        print("Done homing arm with goal end effector angle")

    def home_gantry(self):
        print("Homing gantry")
        self.tbd_controller.home_gantry()
        print("Done homing gantry")

    def home_turntable(self):
        print("Homing turntable")
        self.tbd_controller.home_turntable()
        print("Done homing turntable")

    def print_current_base_position(self):
        self.current_base_position = self.tbd_controller.get_current_base_position()
        print("Current Base Position: [" + str(self.current_base_position[0]) + ", " + str(self.current_base_position[1]) + ", " + str(self.current_base_position[2]) + "]")

    def move_robot_base(self, desired_coords):
        self.print_current_base_position()

        print("Moving Robot Base to Position: [" + str(desired_coords[0]) + ", " + str(desired_coords[1]) + ", " + str(desired_coords[2]) + "]")
        # Move the robot base to the desired coords.
        self.done_moving_robot_base_flag = self.tbd_controller.move_robot_base(desired_coords)
        while(self.done_moving_robot_base_flag != True):
            self.done_moving_robot_base_flag = self.tbd_controller.move_robot_base(desired_coords)

        self.print_current_base_position()


    def move_to_station(self):
        # determine the station base coords
        self.determine_station_base_coords()
    
        self.move_robot_base(self.station_base_coords)

    def turn_turntable(self, desired_turntable_degree):

        self.print_current_turntable_degree()

        print("Turning turntable to degree: " + str(desired_turntable_degree))

        desired_turntable_theta = math.radians(desired_turntable_degree)
        self.done_turning_turntable_flag = self.tbd_controller.turn_turntable(desired_turntable_theta)
        while(self.done_turning_turntable_flag != True):
            self.done_turning_turntable_flag = self.tbd_controller.turn_turntable(desired_turntable_theta)

        self.print_current_turntable_degree()

    def print_current_turntable_degree(self):
        self.current_turntable_degree = self.tbd_controller.get_current_turntable_degree()
        print("Current Turntable Degree: [" + str(self.current_turntable_degree) + "]")

    def turn_turntable_to_station(self):
        # determine the station turntable degree
        self.determine_station_turntable_degree()
        
        self.turn_turntable(self.station_turntable_degree)

    def turn_turntable_to_v3_offset(self):
        self.turn_turntable(self.station_turntable_degree + self.horizontal_station_v3_degree_offset)

    def set_station(self, station):
        self.station = station
        print("Station: " + self.station)

    def set_actuator(self, actuator):
        self.actuator = actuator

        if(self.actuator == "V2" or self.actuator == "A" or self.actuator == "B"):
            self.station_orientation == "vertical"

        print("Actuator: " + self.actuator)

    def set_actuations(self, actuations):
        self.actuations = actuations
        print("Actuations: " + str(self.actuations)[1:-1])

    def start_vision(self):
        print("Starting vision")
        self.tbd_cv = TheBoatDoctorCV(self.actuator, self.station, self.station_orientation)

    def position_arm_for_kinect_vision(self):
        print("Homing arm for Kinect vision")
        self.home_arm()
        print("Homing gantry for Kinect vision")
        self.home_gantry()
        print("Positioning arm for Kinect vision")
        self.tbd_controller.position_arm_for_kinect_vision()

    def determine_breaker_positions_and_orientations_using_kinect(self):
        try:
            (self.breaker_positions_in_3d_camera_coordinates, self.current_breaker_positions) = self.tbd_cv.get_station_info_kinect()

            for i in range(3):
                print("Breaker " + str(i) + " position in 3D camera coordinates: " + str(self.breaker_positions_in_3d_camera_coordinates[3*i:3*(i+1)]))
                print("Breaker " + str(i) + " position: " + self.current_breaker_positions[i])
                self.breaker_positions_in_3d_robot_coordinates[i] = self.cam_to_ik(numpy.array(self.breaker_positions_in_3d_camera_coordinates[3*i:3*(i+1)]))
                print("Station object position in 3D robot coordinates: " + str(self.breaker_positions_in_3d_robot_coordinates))
            return True
        except:
            print("Error with Kinect vision. Moving on to the next station.")
            return False

    def determine_station_position_and_orientation_using_kinect(self):   
        try:
            (self.station_object_position_in_3d_camera_coordinates, self.station_orientation) = self.tbd_cv.get_station_info_kinect()

            print("Station object position in 3D camera coordinates: " + str(self.station_object_position_in_3d_camera_coordinates))
            print("Station orientation: " + self.station_orientation)
            self.station_object_position_in_3d_robot_coordinates = self.cam_to_ik(numpy.array(self.station_object_position_in_3d_camera_coordinates))
            print("Station object position in 3D robot coordinates: " + str(self.station_object_position_in_3d_robot_coordinates))
            return True
        except:
            print("Error with Kinect vision. Moving on to the next station.")
            return False

    ######### TODO: Make Better for all stations
    def generate_robot_trajectory_using_ik(self):
        ## Gate Valves
        if(self.actuator == "V1"):
            self.raspberry_pi_camera_position = self.station_object_position_in_3d_robot_coordinates

            if(self.station_orientation == "vertical"):
                    self.raspberry_pi_camera_position[0] = self.raspberry_pi_camera_position[0] + self.raspberry_pi_camera_vertical_station_v1_x_offset
                    self.raspberry_pi_camera_position[2] = self.raspberry_pi_camera_position[2] + self.raspberry_pi_camera_vertical_station_v1_z_offset
            elif(self.station_orientation == "horizontal"):
                    self.raspberry_pi_camera_position[0] = self.raspberry_pi_camera_position[0] + self.raspberry_pi_camera_horizontal_station_v1_x_offset
                    self.raspberry_pi_camera_position[2] = self.raspberry_pi_camera_position[2] + self.raspberry_pi_camera_horizontal_station_v1_z_offset
            else:
                print("Station orientation was not provided.")

            self.joint_angles_for_raspberry_pi_camera_position = numpy.zeros([1,6])

            print("Raspberry Pi Camera Position: " + str(self.raspberry_pi_camera_position))

            self.joint_angles_for_raspberry_pi_camera_position = self.tbd_ik.solve_ik(self.raspberry_pi_camera_position, self.station_orientation)
            
            if(abs(self.joint_angles_for_raspberry_pi_camera_position[1]) < 0.000001):
                self.joint_angles_for_raspberry_pi_camera_position[1] = 0.0
            if(abs(self.joint_angles_for_raspberry_pi_camera_position[2]) < 0.000001):
                self.joint_angles_for_raspberry_pi_camera_position[2] = 0.0

            self.joint_angles_for_goal_position = numpy.copy(self.joint_angles_for_raspberry_pi_camera_position)

            if(self.station_orientation == "vertical"):
                self.joint_angles_for_goal_position[1] = self.joint_angles_for_goal_position[1] - self.raspberry_pi_camera_vertical_station_v1_x_offset + self.vertical_station_v1_goal_x_offset
                self.joint_angles_for_goal_position[2] = self.joint_angles_for_goal_position[2] - self.raspberry_pi_camera_vertical_station_v1_z_offset + self.vertical_station_v1_goal_z_offset
            else:
                self.joint_angles_for_goal_position[1] = self.joint_angles_for_goal_position[1] - self.raspberry_pi_camera_horizontal_station_v1_x_offset + self.horizontal_station_v1_goal_x_offset
                self.joint_angles_for_goal_position[2] = self.joint_angles_for_goal_position[2] - self.raspberry_pi_camera_horizontal_station_v1_z_offset + self.horizontal_station_v1_goal_z_offset

        ## Orange Valve
        elif(self.actuator == "V2"):
            self.raspberry_pi_camera_position = self.station_object_position_in_3d_robot_coordinates

            if(self.station_orientation == "vertical"):
                self.raspberry_pi_camera_position[0] = self.raspberry_pi_camera_position[0] + self.raspberry_pi_camera_vertical_station_v2_x_offset
                self.raspberry_pi_camera_position[2] = self.raspberry_pi_camera_position[2] + self.raspberry_pi_camera_vertical_station_v2_z_offset
            else:
                print("Station orientation was not provided.")

            self.joint_angles_for_raspberry_pi_camera_position = numpy.zeros([1,6])

            print("Raspberry Pi Camera Position: " + str(self.raspberry_pi_camera_position))

            self.joint_angles_for_raspberry_pi_camera_position = self.tbd_ik.solve_ik(self.raspberry_pi_camera_position, self.station_orientation)
            
            if(abs(self.joint_angles_for_raspberry_pi_camera_position[1]) < 0.000001):
                self.joint_angles_for_raspberry_pi_camera_position[1] = 0.0
            if(abs(self.joint_angles_for_raspberry_pi_camera_position[2]) < 0.000001):
                self.joint_angles_for_raspberry_pi_camera_position[2] = 0.0

            self.joint_angles_for_goal_position = numpy.copy(self.joint_angles_for_raspberry_pi_camera_position)

            self.joint_angles_for_goal_position[1] = self.joint_angles_for_goal_position[1] - self.raspberry_pi_camera_vertical_station_v2_x_offset + self.vertical_station_v2_goal_x_offset
            self.joint_angles_for_goal_position[2] = self.joint_angles_for_goal_position[2] - self.raspberry_pi_camera_vertical_station_v2_z_offset + self.vertical_station_v2_goal_z_offset
            
        ## Shuttlecock Valves    
        elif(self.actuator == "V3"):
            self.raspberry_pi_camera_position = self.station_object_position_in_3d_robot_coordinates

            if(self.station_orientation == "vertical"):
                    self.raspberry_pi_camera_position[0] = self.raspberry_pi_camera_position[0] + self.raspberry_pi_camera_vertical_station_v3_x_offset
                    self.raspberry_pi_camera_position[2] = self.raspberry_pi_camera_position[2] + self.raspberry_pi_camera_vertical_station_v3_z_offset
            elif(self.station_orientation == "horizontal"):
                if(self.station == "E"):
                    self.raspberry_pi_camera_position[0] = self.raspberry_pi_camera_position[0] + self.corner_e_raspberry_pi_camera_horizontal_station_v3_x_offset
                    self.raspberry_pi_camera_position[2] = self.raspberry_pi_camera_position[2] + self.corner_e_raspberry_pi_camera_horizontal_station_v3_z_offset
                else:
                    self.raspberry_pi_camera_position[0] = self.raspberry_pi_camera_position[0] + self.raspberry_pi_camera_horizontal_station_v3_x_offset
                    self.raspberry_pi_camera_position[2] = self.raspberry_pi_camera_position[2] + self.raspberry_pi_camera_horizontal_station_v3_z_offset
            else:
                print("Station orientation was not provided.")

            self.joint_angles_for_raspberry_pi_camera_position = numpy.zeros([1,6])

            print("Raspberry Pi Camera Position: " + str(self.raspberry_pi_camera_position))

            self.joint_angles_for_raspberry_pi_camera_position = self.tbd_ik.solve_ik(self.raspberry_pi_camera_position, self.station_orientation)
            
            if(abs(self.joint_angles_for_raspberry_pi_camera_position[1]) < 0.000001):
                self.joint_angles_for_raspberry_pi_camera_position[1] = 0.0
            if(abs(self.joint_angles_for_raspberry_pi_camera_position[2]) < 0.000001):
                self.joint_angles_for_raspberry_pi_camera_position[2] = 0.0

            self.joint_angles_for_goal_position = numpy.copy(self.joint_angles_for_raspberry_pi_camera_position)

            if(self.station_orientation == "vertical"):
                self.joint_angles_for_goal_position[1] = self.joint_angles_for_goal_position[1] - self.raspberry_pi_camera_vertical_station_v3_x_offset + self.vertical_station_v3_goal_x_offset
                self.joint_angles_for_goal_position[2] = self.joint_angles_for_goal_position[2] - self.raspberry_pi_camera_vertical_station_v3_z_offset + self.vertical_station_v3_goal_z_offset
            
        ## Breakers
        elif(self.actuator == "A" or self.actuator == "B"):
            
            self.joint_angles_for_goal_positions = numpy.zeros([self.num_breakers_to_actuate,6])

            if(self.station == "E"):
                for i in range(self.num_breakers_to_actuate):
                    breaker_index = 0
                    corner_e_breaker_x_offset = 0.0
                    corner_e_breaker_z_offset = 0.0
                    if(self.breaker_commands[2*i] == "B1"):
                        breaker_index = 0
                        if(self.breaker_commands[2*i+1] == "U"):
                            corner_e_breaker_x_offset = self.corner_e_breaker_1_b_x_offset
                            corner_e_breaker_z_offset = self.corner_e_breaker_1_b_z_offset
                        else:
                            corner_e_breaker_x_offset = self.corner_e_breaker_1_a_x_offset
                            corner_e_breaker_z_offset = self.corner_e_breaker_1_a_z_offset

                    elif(self.breaker_commands[2*i] == "B2"):
                        breaker_index = 1
                        if(self.breaker_commands[2*i+1] == "U"):
                            corner_e_breaker_x_offset = self.corner_e_breaker_2_b_x_offset
                            corner_e_breaker_z_offset = self.corner_e_breaker_2_b_z_offset
                        else:
                            corner_e_breaker_x_offset = self.corner_e_breaker_2_a_x_offset
                            corner_e_breaker_z_offset = self.corner_e_breaker_2_a_z_offset

                    elif(self.breaker_commands[2*i] == "B3"):
                        breaker_index = 2
                        if(self.breaker_commands[2*i+1] == "U"):
                            corner_e_breaker_x_offset = self.corner_e_breaker_3_b_x_offset
                            corner_e_breaker_z_offset = self.corner_e_breaker_3_b_z_offset
                        else:
                            corner_e_breaker_x_offset = self.corner_e_breaker_3_a_x_offset
                            corner_e_breaker_z_offset = self.corner_e_breaker_3_a_z_offset
                    else:
                        print("Invalid breaker in mission file")
                        break
                    breaker_goal_position = self.breaker_positions_in_3d_robot_coordinates[breaker_index]
                    breaker_goal_position[0] = breaker_goal_position[0] + corner_e_breaker_x_offset
                    breaker_goal_position[2] = breaker_goal_position[2] + corner_e_breaker_z_offset
                    print("Breaker goal position: [ " + str(breaker_goal_position[0]) + ", " + str(breaker_goal_position[1]) + ", " + str(breaker_goal_position[2]) + "]")
                    self.joint_angles_for_goal_positions[i] = self.tbd_ik.solve_ik(breaker_goal_position, "vertical")
            
                    if(abs(self.joint_angles_for_goal_positions[i][1]) < 0.000001):
                        self.joint_angles_for_goal_positions[i][1] = 0.0
                    if(abs(self.joint_angles_for_goal_positions[i][2]) < 0.000001):
                        self.joint_angles_for_goal_positions[i][2] = 0.0
            elif(self.station == "F"):
                for i in range(self.num_breakers_to_actuate):
                    breaker_index = 0
                    corner_f_breaker_x_offset = 0.0
                    corner_f_breaker_z_offset = 0.0
                    if(self.breaker_commands[2*i] == "B1"):
                        breaker_index = 0
                        if(self.breaker_commands[2*i+1] == "U"):
                            corner_f_breaker_x_offset = self.corner_f_breaker_1_b_x_offset
                            corner_f_breaker_z_offset = self.corner_f_breaker_1_b_z_offset
                        else:
                            corner_f_breaker_x_offset = self.corner_f_breaker_1_a_x_offset
                            corner_f_breaker_z_offset = self.corner_f_breaker_1_a_z_offset

                    elif(self.breaker_commands[2*i] == "B2"):
                        breaker_index = 1
                        if(self.breaker_commands[2*i+1] == "U"):
                            corner_f_breaker_x_offset = self.corner_f_breaker_2_b_x_offset
                            corner_f_breaker_z_offset = self.corner_f_breaker_2_b_z_offset
                        else:
                            corner_f_breaker_x_offset = self.corner_f_breaker_2_a_x_offset
                            corner_f_breaker_z_offset = self.corner_f_breaker_2_a_z_offset

                    elif(self.breaker_commands[2*i] == "B3"):
                        breaker_index = 2
                        if(self.breaker_commands[2*i+1] == "U"):
                            corner_f_breaker_x_offset = self.corner_f_breaker_3_b_x_offset
                            corner_f_breaker_z_offset = self.corner_f_breaker_3_b_z_offset
                        else:
                            corner_f_breaker_x_offset = self.corner_f_breaker_3_a_x_offset
                            corner_f_breaker_z_offset = self.corner_f_breaker_3_a_z_offset
                    else:
                        print("Invalid breaker in mission file")
                        break
                    breaker_goal_position = self.breaker_positions_in_3d_robot_coordinates[breaker_index]
                    breaker_goal_position[0] = breaker_goal_position[0] + corner_f_breaker_x_offset
                    breaker_goal_position[2] = breaker_goal_position[2] + corner_f_breaker_z_offset
                    print("Breaker goal position: [ " + str(breaker_goal_position[0]) + ", " + str(breaker_goal_position[1]) + ", " + str(breaker_goal_position[2]) + "]")
                    self.joint_angles_for_goal_positions[i] = self.tbd_ik.solve_ik(breaker_goal_position, "vertical")
            
                    if(abs(self.joint_angles_for_goal_positions[i][1]) < 0.000001):
                        self.joint_angles_for_goal_positions[i][1] = 0.0
                    if(abs(self.joint_angles_for_goal_positions[i][2]) < 0.000001):
                        self.joint_angles_for_goal_positions[i][2] = 0.0
            else:
                for i in range(self.num_breakers_to_actuate):
                    breaker_index = 0
                    breaker_x_offset = 0.0
                    breaker_z_offset = 0.0
                    if(self.breaker_commands[2*i] == "B1"):
                        breaker_index = 0
                        if(self.breaker_commands[2*i+1] == "U"):
                            breaker_x_offset = self.breaker_1_b_x_offset
                            breaker_z_offset = self.breaker_1_b_z_offset
                        else:
                            breaker_x_offset = self.breaker_1_a_x_offset
                            breaker_z_offset = self.breaker_1_a_z_offset
                    elif(self.breaker_commands[2*i] == "B2"):
                        breaker_index = 1
                        if(self.breaker_commands[2*i+1] == "U"):
                            breaker_x_offset = self.breaker_2_b_x_offset
                            breaker_z_offset = self.breaker_2_b_z_offset
                        else:
                            breaker_x_offset = self.breaker_2_a_x_offset
                            breaker_z_offset = self.breaker_2_a_z_offset
                    elif(self.breaker_commands[2*i] == "B3"):
                        breaker_index = 2
                        if(self.breaker_commands[2*i+1] == "U"):
                            breaker_x_offset = self.breaker_3_b_x_offset
                            breaker_z_offset = self.breaker_3_b_z_offset
                        else:
                            breaker_x_offset = self.breaker_3_a_x_offset
                            breaker_z_offset = self.breaker_3_a_z_offset
                    else:
                        print("Invalid breaker in mission file")
                        break
                    breaker_goal_position = self.breaker_positions_in_3d_robot_coordinates[breaker_index]
                    breaker_goal_position[0] = breaker_goal_position[0] + breaker_x_offset
                    breaker_goal_position[2] = breaker_goal_position[2] + breaker_z_offset
                    print("Breaker goal position: [ " + str(breaker_goal_position[0]) + ", " + str(breaker_goal_position[1]) + ", " + str(breaker_goal_position[2]) + "]")
                    self.joint_angles_for_goal_positions[i] = self.tbd_ik.solve_ik(breaker_goal_position, "vertical")
                              
                    if(abs(self.joint_angles_for_goal_positions[i][1]) < 0.000001):
                        self.joint_angles_for_goal_positions[i][1] = 0.0
                    if(abs(self.joint_angles_for_goal_positions[i][2]) < 0.000001):
                        self.joint_angles_for_goal_positions[i][2] = 0.0

    def print_current_gantry_position(self):
        self.current_gantry_position = self.tbd_controller.get_current_gantry_position()
        print("Current Gantry Position: [" + str(self.current_gantry_position[0]) + ", " + str(self.current_gantry_position[1]) + "]")

    def move_gantry(self, desired_positions):
        self.print_current_gantry_position()

        print("Desired Gantry Position: " + str(desired_positions))
        self.done_moving_gantry_flag = self.tbd_controller.move_gantry(desired_positions)
        while(self.done_moving_gantry_flag != True):
                self.done_moving_gantry_flag = self.tbd_controller.move_gantry(desired_positions)

        self.print_current_gantry_position()

    def move_x_gantry_to_center(self):
        self.print_current_gantry_position()

        desired_positions = [0.11, self.current_gantry_position[1]]

        print("Desired Gantry Position: " + str(desired_positions))
        self.done_moving_gantry_flag = self.tbd_controller.move_gantry(desired_positions)
        while(self.done_moving_gantry_flag != True):
                self.done_moving_gantry_flag = self.tbd_controller.move_gantry(desired_positions)

        self.print_current_gantry_position()

    def move_arm(self, desired_arm_thetas):
        print("Desired Arm Angles: " + str(desired_arm_thetas))
        self.done_moving_arm_flag = self.tbd_controller.move_arm(desired_arm_thetas)
        while(self.done_moving_arm_flag != True):
            self.done_moving_arm_flag = self.tbd_controller.move_arm(desired_arm_thetas)

    def move_to_raspberry_pi_camera_position(self):
        self.home_arm()

        ## Move Gantry to Raspberry Pi Camera location
        self.move_gantry([self.joint_angles_for_raspberry_pi_camera_position[1], self.joint_angles_for_raspberry_pi_camera_position[2]])

        ## Move Arm to Raspberry Pi Camera location
        self.move_arm([self.joint_angles_for_raspberry_pi_camera_position[3], self.joint_angles_for_raspberry_pi_camera_position[4], self.joint_angles_for_raspberry_pi_camera_position[5]])
            
    def determine_station_orientation_using_raspberry_pi_camera(self):
        ## Get the current angle of the station
        self.current_station_angle_in_degrees = float(self.tbd_cv.get_station_info_pi())
        # num_tries = 1
        # while(self.current_station_angle_in_degrees == 10000 and num_tries < 3):
        #     num_tries = num_tries + 1
        #     self.current_station_angle_in_degrees = float(self.tbd_cv.get_station_info_pi())

        if(self.current_station_angle_in_degrees == 10000):
            self.turn_on_leds()
            self.turn_off_leds()
            self.turn_on_leds()
            self.turn_off_leds()
            return False

        ## Shuttlecock Valves Only
        if(self.actuator == "V3"):
            if(abs(90 - abs(self.current_station_angle_in_degrees)) < self.v3_valve_threshold):
                self.current_station_angle_in_degrees = 90
            elif(abs(self.current_station_angle_in_degrees) < self.v3_valve_threshold):
                self.current_station_angle_in_degrees = 0

            if(self.station_orientation == "vertical"):
                self.current_station_angle_in_degrees = -self.current_station_angle_in_degrees

        print("Current Station Angle: " + str(self.current_station_angle_in_degrees))
        return True

    def determine_mission_goal(self):
        ## Breakers
        if(self.actuator == "A" or self.actuator == "B"):
            self.desired_breaker_positions = numpy.copy(self.current_breaker_positions)

            self.num_breakers_in_task = len(self.actuations) / 2

            print("Num Breakers in task: " + str(self.num_breakers_in_task))

            for i in range(self.num_breakers_in_task):
                breaker_index = 0
                if(self.actuations[2*i] == "B1"):
                    breaker_index = 0
                elif(self.actuations[2*i] == "B2"):
                    breaker_index = 1
                elif(self.actuations[2*i] == "B3"):
                    breaker_index = 2
                else:
                    print("Invalid breaker in mission file")
                    return False
                print("Desired breaker position: " + self.actuations[2*i+1])
                self.desired_breaker_positions[breaker_index] = self.actuations[2*i+1]

            print("Desired Breaker Positions: " + str(self.desired_breaker_positions))

        else:   
            ## Gate and Orange Valves 
            if(self.actuator == "V1" or self.actuator == "V2"):
                self.direction = self.actuations[0][0]
                self.degree = float(self.actuations[0][1:])
                if(self.direction == "+"):
                    self.desired_station_angle_in_degrees = ((self.current_station_angle_in_degrees + self.degree + 180) % 360) - 180 

                elif(self.direction == "-"):
                    self.desired_station_angle_in_degrees = ((self.current_station_angle_in_degrees - self.degree + 180) % 360) - 180 

            ## Shuttlecock Valves
            elif(self.actuator == "V3"):
                self.desired_station_position = self.actuations[0][0]

                if(self.station_orientation == "vertical"):
                    # Open
                    if(self.desired_station_position == "0"):
                        self.desired_station_angle_in_degrees = -90

                    # Closed
                    elif(self.desired_station_position == "1"):
                        self.desired_station_angle_in_degrees = 0

                elif(self.station_orientation == "horizontal"):
                    # Openself.horizontal_station_v3_goal_z_gantry_offset = 0.05
                    if(self.desired_station_position == "0"):
                        self.desired_station_angle_in_degrees = 0

                    # Closed
                    elif(self.desired_station_position == "1"):
                        self.desired_station_angle_in_degrees = 90

            print("Desired Station Angle: " + str(self.desired_station_angle_in_degrees))

    def update_waypoints_with_mission_goal_from_mission_file(self):
        ## Gate and Orange Valves
        if(self.actuator == "V1" or self.actuator == "V2"):
            self.direction = self.actuations[0][0]
            self.degree = float(self.actuations[0][1:])
            if(self.direction == "+"):
                self.desired_end_effector_angle_in_radians = self.degree * math.pi / 180

            elif(self.direction == "-"):
                self.desired_end_effector_angle_in_radians = -self.degree * math.pi / 180

        ## Shuttlecock Valves
        elif(self.actuator == "V3"):
            self.update_waypoints_with_mission_goal()

            if(self.station_orientation == "horizontal"): 
                goal_position = self.station_object_position_in_3d_robot_coordinates

                if(self.station == "E"):
                    if(self.desired_end_effector_angle_in_radians > 0):
                        
                        goal_position[0] = goal_position[0] + self.corner_e_horizontal_station_closed_v3_goal_x_offset
                        goal_position[2] = goal_position[2] + self.corner_e_horizontal_station_closed_v3_goal_z_offset
                        self.joint_angles_for_goal_position = self.tbd_ik.solve_ik(goal_position, "vertical")

                        self.joint_angles_for_x_gantry_actuation_position = numpy.copy(self.joint_angles_for_goal_position)
                        self.joint_angles_for_goal_position_2 = numpy.copy(self.joint_angles_for_goal_position)
                        self.joint_angles_for_goal_position_2[1] = self.joint_angles_for_goal_position_2[1] + self.corner_e_horizontal_station_closed_v3_goal_2_x_offset
                        self.joint_angles_for_goal_position_2[2] = self.joint_angles_for_goal_position_2[2] + self.corner_e_horizontal_station_closed_v3_goal_2_z_offset
                    
                        self.joint_angles_for_x_gantry_actuation_position[1] = self.joint_angles_for_x_gantry_actuation_position[1] - self.corner_e_horizontal_station_closed_v3_goal_x_gantry_offset 

                        if(self.joint_angles_for_x_gantry_actuation_position[1] <= 0.0):
                            self.joint_angles_for_x_gantry_actuation_position[1] = 0.0

                        self.desired_wrist_angles_in_radians = [self.corner_e_horizontal_station_closed_v3_wrist_flip_theta]
                    else:
                        goal_position[0] = goal_position[0] + self.corner_e_horizontal_station_open_v3_goal_x_offset
                        goal_position[2] = goal_position[2] + self.corner_e_horizontal_station_open_v3_goal_z_offset
                        self.joint_angles_for_goal_position = self.tbd_ik.solve_ik(goal_position, "vertical")

                        self.desired_end_effector_angle_in_radians = -2*math.pi

                        self.joint_angles_for_x_gantry_actuation_position = numpy.copy(self.joint_angles_for_goal_position)
                        
                        self.joint_angles_for_x_gantry_actuation_position[1] = self.joint_angles_for_x_gantry_actuation_position[1] + self.corner_e_horizontal_station_open_v3_goal_x_gantry_offset 

                        self.joint_angles_for_x_gantry_actuation_position[5] = self.desired_end_effector_angle_in_radians
                else:
                    if(self.desired_end_effector_angle_in_radians > 0):
                        
                        goal_position[0] = goal_position[0] + self.horizontal_station_closed_v3_goal_x_offset
                        goal_position[2] = goal_position[2] + self.horizontal_station_closed_v3_goal_z_offset
                        self.joint_angles_for_goal_position = self.tbd_ik.solve_ik(goal_position, "vertical")

                        self.joint_angles_for_x_gantry_actuation_position = numpy.copy(self.joint_angles_for_goal_position)
                        self.joint_angles_for_goal_position_2 = numpy.copy(self.joint_angles_for_goal_position)
                        self.joint_angles_for_goal_position_2[1] = self.joint_angles_for_goal_position_2[1] + self.horizontal_station_closed_v3_goal_2_x_offset
                        self.joint_angles_for_goal_position_2[2] = self.joint_angles_for_goal_position_2[2] + self.horizontal_station_closed_v3_goal_2_z_offset
                    
                        self.joint_angles_for_x_gantry_actuation_position[1] = self.joint_angles_for_x_gantry_actuation_position[1] - self.horizontal_station_closed_v3_goal_x_gantry_offset 

                        if(self.joint_angles_for_x_gantry_actuation_position[1] <= 0.0):
                            self.joint_angles_for_x_gantry_actuation_position[1] = 0.0

                        self.desired_wrist_angles_in_radians = [self.horizontal_station_closed_v3_wrist_flip_theta]
                    else:
                        goal_position[0] = goal_position[0] + self.horizontal_station_open_v3_goal_x_offset
                        goal_position[2] = goal_position[2] + self.horizontal_station_open_v3_goal_z_offset
                        self.joint_angles_for_goal_position = self.tbd_ik.solve_ik(goal_position, "vertical")

                        self.desired_end_effector_angle_in_radians = -2*math.pi

                        self.joint_angles_for_x_gantry_actuation_position = numpy.copy(self.joint_angles_for_goal_position)
                        
                        self.joint_angles_for_x_gantry_actuation_position[1] = self.joint_angles_for_x_gantry_actuation_position[1] + self.horizontal_station_open_v3_goal_x_gantry_offset 

                        self.joint_angles_for_x_gantry_actuation_position[5] = self.desired_end_effector_angle_in_radians
                    # self.joint_angles_for_goal_position = numpy.copy(self.joint_angles_for_raspberry_pi_camera_position)

                    # self.joint_angles_for_goal_position[1] = self.joint_angles_for_goal_position[1] + self.horizontal_station_open_v3_goal_x_offset

                    # self.joint_angles_for_goal_position[2] = self.joint_angles_for_goal_position[2] + self.horizontal_station_open_v3_goal_z_offset

                    # self.joint_angles_for_goal_position_2 = self.joint_angles_for_goal_position

    def update_waypoints_with_mission_goal(self):
        ## Breakers
        if(self.actuator == "A" or self.actuator == "B"):
            self.desired_wrist_angles_in_radians = []
            for i in range(self.num_breakers_to_actuate):
                if(self.breaker_commands[2*i + 1] == "U"):
                    self.desired_wrist_angles_in_radians.append(self.breaker_b_goal_wrist_flip_up_theta)
                
                    self.desired_end_effector_angle_in_radians = 0.0
                else:
                    if(self.actuator == "B"):
                        self.desired_wrist_angles_in_radians.append(self.breaker_b_goal_wrist_flip_down_theta)
                        self.joint_angles_for_goal_positions[i][5] = math.pi
                        self.desired_end_effector_angle_in_radians = math.pi
                        self.desired_z_gantry_movement = -0.05;
                    else:
                        self.desired_wrist_angles_in_radians.append(self.breaker_a_goal_wrist_flip_down_theta)
                        self.desired_end_effector_angle_in_radians = 0.0
        else:
            if(abs(((self.desired_station_angle_in_degrees - self.current_station_angle_in_degrees + 180) % 360) - 180) < self.angle_threshold_in_degrees):
                return
            else:
                self.desired_end_effector_angle_in_radians = (((self.desired_station_angle_in_degrees - self.current_station_angle_in_degrees + 180) % 360) - 180) * math.pi / 180

            if(self.actuator == "V3" and self.station_orientation == "vertical"):
                if(self.desired_end_effector_angle_in_radians > 0):
                    self.desired_end_effector_angle_in_radians = self.desired_end_effector_angle_in_radians + (math.pi / 4)
                else:
                    self.desired_end_effector_angle_in_radians = self.desired_end_effector_angle_in_radians - (math.pi / 4)
            
    def move_to_station_object(self):
        self.home_arm()

        ## Move Arm and Gantry towards the station
        if(self.station_orientation == "vertical"):
            self.move_gantry([self.joint_angles_for_raspberry_pi_camera_position[1], self.joint_angles_for_goal_position[2]])
            self.move_arm([self.joint_angles_for_goal_position[3], self.joint_angles_for_goal_position[4], self.joint_angles_for_goal_position[5]])
            self.move_gantry([self.joint_angles_for_goal_position[1], self.joint_angles_for_goal_position[2]])
        else:
            self.move_gantry([self.joint_angles_for_raspberry_pi_camera_position[1], self.joint_angles_for_goal_position[2]])
            self.move_gantry([self.joint_angles_for_goal_position[1], self.joint_angles_for_goal_position[2]])
            self.move_arm([self.joint_angles_for_goal_position[3], self.joint_angles_for_goal_position[4], self.joint_angles_for_goal_position[5]])

    def move_to_breaker(self, breaker_command_index):
        self.home_arm()

        self.home_gantry()
        
        breaker_to_actuate = self.breaker_commands[2*breaker_command_index]

        if(self.station == "E"):
            if(breaker_to_actuate == "B1"):
                if(self.actuator == "A"):
                    self.turn_turntable(self.station_turntable_degree + self.corner_e_breaker_1_a_turntable_degree_offset)
                else:
                    self.turn_turntable(self.station_turntable_degree + self.corner_e_breaker_1_b_turntable_degree_offset)
                self.move_gantry([0.0, self.joint_angles_for_goal_positions[breaker_command_index][2]])
                self.move_arm([self.joint_angles_for_goal_positions[breaker_command_index][3], self.joint_angles_for_goal_positions[breaker_command_index][4], self.joint_angles_for_goal_positions[breaker_command_index][5]])
                self.move_gantry([self.joint_angles_for_goal_positions[breaker_command_index][1], self.joint_angles_for_goal_positions[breaker_command_index][2]])
                self.joint_angles_for_goal_position_2 = numpy.copy(self.joint_angles_for_goal_positions[breaker_command_index][:])

            elif(breaker_to_actuate == "B2"):
                if(self.actuator == "A"):
                    self.turn_turntable(self.station_turntable_degree + self.corner_e_breaker_2_a_turntable_degree_offset)
                else:
                    self.turn_turntable(self.station_turntable_degree + self.corner_e_breaker_2_b_turntable_degree_offset)
                self.move_gantry([0.0, self.joint_angles_for_goal_positions[breaker_command_index][2]])
                self.move_arm([self.joint_angles_for_goal_positions[breaker_command_index][3], self.joint_angles_for_goal_positions[breaker_command_index][4], self.joint_angles_for_goal_positions[breaker_command_index][5]])
                self.move_gantry([self.joint_angles_for_goal_positions[breaker_command_index][1], self.joint_angles_for_goal_positions[breaker_command_index][2]])
                self.joint_angles_for_goal_position_2 = numpy.copy(self.joint_angles_for_goal_positions[breaker_command_index][:])

            elif(breaker_to_actuate == "B3"):
                if(self.actuator == "A"):
                    self.turn_turntable(self.station_turntable_degree + self.corner_e_breaker_3_a_turntable_degree_offset)
                else:
                    self.turn_turntable(self.station_turntable_degree + self.corner_e_breaker_3_b_turntable_degree_offset)
                self.move_gantry([0.0, self.joint_angles_for_goal_positions[breaker_command_index][2]])
                self.move_arm([self.joint_angles_for_goal_positions[breaker_command_index][3], self.joint_angles_for_goal_positions[breaker_command_index][4], self.joint_angles_for_goal_positions[breaker_command_index][5]])
                self.move_gantry([self.joint_angles_for_goal_positions[breaker_command_index][1], self.joint_angles_for_goal_positions[breaker_command_index][2]])
                self.joint_angles_for_goal_position_2 = numpy.copy(self.joint_angles_for_goal_positions[breaker_command_index][:])

        elif(self.station == "F"):
            if(breaker_to_actuate == "B1"):
                if(self.actuator == "A"):
                    self.turn_turntable(self.station_turntable_degree + self.corner_f_breaker_1_a_turntable_degree_offset)
                else:
                    self.turn_turntable(self.station_turntable_degree + self.corner_f_breaker_1_b_turntable_degree_offset)
                
                self.move_gantry([0.0, self.joint_angles_for_goal_positions[breaker_command_index][2]])
                self.move_arm([self.joint_angles_for_goal_positions[breaker_command_index][3], self.joint_angles_for_goal_positions[breaker_command_index][4], self.joint_angles_for_goal_positions[breaker_command_index][5]])
                self.move_gantry([self.joint_angles_for_goal_positions[breaker_command_index][1], self.joint_angles_for_goal_positions[breaker_command_index][2]])
                self.joint_angles_for_goal_position_2 = numpy.copy(self.joint_angles_for_goal_positions[breaker_command_index][:])

            elif(breaker_to_actuate == "B2"):
                if(self.actuator == "A"):
                    self.turn_turntable(self.station_turntable_degree + self.corner_f_breaker_2_a_turntable_degree_offset)
                else:
                    self.turn_turntable(self.station_turntable_degree + self.corner_f_breaker_2_b_turntable_degree_offset)
                self.move_gantry([0.0, self.joint_angles_for_goal_positions[breaker_command_index][2]])
                self.move_arm([self.joint_angles_for_goal_positions[breaker_command_index][3], self.joint_angles_for_goal_positions[breaker_command_index][4], self.joint_angles_for_goal_positions[breaker_command_index][5]])
                self.move_gantry([self.joint_angles_for_goal_positions[breaker_command_index][1], self.joint_angles_for_goal_positions[breaker_command_index][2]])
                self.joint_angles_for_goal_position_2 = numpy.copy(self.joint_angles_for_goal_positions[breaker_command_index][:])

            elif(breaker_to_actuate == "B3"):
                if(self.actuator == "A"):
                    self.turn_turntable(self.station_turntable_degree + self.corner_f_breaker_3_a_turntable_degree_offset)
                else:
                    self.turn_turntable(self.station_turntable_degree + self.corner_f_breaker_3_b_turntable_degree_offset)
                self.move_gantry([0.0, self.joint_angles_for_goal_positions[breaker_command_index][2]])
                self.move_arm([self.joint_angles_for_goal_positions[breaker_command_index][3], self.joint_angles_for_goal_positions[breaker_command_index][4], self.joint_angles_for_goal_positions[breaker_command_index][5]])
                self.move_gantry([self.joint_angles_for_goal_positions[breaker_command_index][1], self.joint_angles_for_goal_positions[breaker_command_index][2]])
                self.joint_angles_for_goal_position_2 = numpy.copy(self.joint_angles_for_goal_positions[breaker_command_index][:])

        else:
            if(breaker_to_actuate == "B1"):
                self.turn_turntable(self.station_turntable_degree + self.breaker_1_turntable_degree_offset)
                self.move_gantry([0.0, self.joint_angles_for_goal_positions[breaker_command_index][2]])
                self.move_arm([self.joint_angles_for_goal_positions[breaker_command_index][3], self.joint_angles_for_goal_positions[breaker_command_index][4], self.joint_angles_for_goal_positions[breaker_command_index][5]])
                self.move_gantry([self.joint_angles_for_goal_positions[breaker_command_index][1], self.joint_angles_for_goal_positions[breaker_command_index][2]])
                self.joint_angles_for_goal_position_2 = numpy.copy(self.joint_angles_for_goal_positions[breaker_command_index][:])

            elif(breaker_to_actuate == "B2"):
                self.turn_turntable(self.station_turntable_degree + self.breaker_2_turntable_degree_offset)
                self.move_gantry([0.0, self.joint_angles_for_goal_positions[breaker_command_index][2]])
                self.move_arm([self.joint_angles_for_goal_positions[breaker_command_index][3], self.joint_angles_for_goal_positions[breaker_command_index][4], self.joint_angles_for_goal_positions[breaker_command_index][5]])
                self.move_gantry([self.joint_angles_for_goal_positions[breaker_command_index][1], self.joint_angles_for_goal_positions[breaker_command_index][2]])
                self.joint_angles_for_goal_position_2 = numpy.copy(self.joint_angles_for_goal_positions[breaker_command_index][:])

            elif(breaker_to_actuate == "B3"):
                self.turn_turntable(self.station_turntable_degree + self.breaker_3_turntable_degree_offset)
                self.move_gantry([0.0, self.joint_angles_for_goal_positions[breaker_command_index][2]])
                self.move_arm([self.joint_angles_for_goal_positions[breaker_command_index][3], self.joint_angles_for_goal_positions[breaker_command_index][4], self.joint_angles_for_goal_positions[breaker_command_index][5]])
                self.move_gantry([self.joint_angles_for_goal_positions[breaker_command_index][1], self.joint_angles_for_goal_positions[breaker_command_index][2]])
                self.joint_angles_for_goal_position_2 = numpy.copy(self.joint_angles_for_goal_positions[breaker_command_index][:])

    def move_gantry_to_actuate_shuttlecock_valve(self):
        self.move_gantry([self.joint_angles_for_x_gantry_actuation_position[1], self.joint_angles_for_x_gantry_actuation_position[2]])

    def move_gantry_to_actuate_breaker(self):
        self.move_gantry([self.joint_angles_for_goal_position_2[1], self.joint_angles_for_goal_position_2[2] + self.desired_z_gantry_movement])

    def move_to_shuttlecock_valve(self):
        self.home_arm()

        ## Move Arm and Gantry towards the station
        self.move_gantry([self.joint_angles_for_x_gantry_actuation_position[1], self.joint_angles_for_goal_position_2[2]])
        self.move_gantry([self.joint_angles_for_goal_position_2[1], self.joint_angles_for_goal_position_2[2]])
        self.move_arm([self.joint_angles_for_goal_position_2[3], self.joint_angles_for_goal_position_2[4], self.joint_angles_for_goal_position_2[5]])

    def actuate_wrist(self, wrist_angle_index):
        self.move_arm([self.joint_angles_for_goal_position_2[3], self.joint_angles_for_goal_position_2[4] + self.desired_wrist_angles_in_radians[wrist_angle_index], self.joint_angles_for_goal_position_2[5]])

    def actuate_elbow(self, elbow_angle_index):
        self.move_arm([self.joint_angles_for_goal_position_2[3] + self.desired_wrist_angles_in_radians[elbow_angle_index], self.joint_angles_for_goal_position_2[4], self.joint_angles_for_goal_position_2[5]])

    def actuate_end_effector(self):
        self.move_arm([self.joint_angles_for_goal_position[3], self.joint_angles_for_goal_position[4], self.desired_end_effector_angle_in_radians])

    def turn_on_leds(self):
        print("Turning on LEDs")
        led_switch_flag = self.tbd_controller.led_switch("on")
        while(led_switch_flag == False):
            led_switch_flag = self.tbd_controller.led_switch("on")
        time.sleep(1)

    def turn_off_leds(self):
        print("Turning off LEDs")
        led_switch_flag = self.tbd_controller.led_switch("off")
        while(led_switch_flag == False):
            led_switch_flag = self.tbd_controller.led_switch("off")
        time.sleep(1)

    def turn_on_pump(self):
        print("Turning on pump")
        pump_switch_flag = self.tbd_controller.pump_switch("on")
        while(pump_switch_flag == False):
            pump_switch_flag = self.tbd_controller.pump_switch("on")
        time.sleep(1)

    def turn_off_pump(self):
        print("Turning off pump")
        pump_switch_flag = self.tbd_controller.pump_switch("off")
        while(pump_switch_flag == False):
            pump_switch_flag = self.tbd_controller.pump_switch("off")
        time.sleep(1)

    def return_to_raspberry_pi_camera_position(self):

        ## Home the arm and move the X Gantry back to the raspberry pi camera position
        if(self.station_orientation == "vertical"):
            self.move_gantry([self.joint_angles_for_raspberry_pi_camera_position[1], self.joint_angles_for_goal_position[2]])
            self.home_arm()
        else:
            self.home_arm_with_goal_end_effector_angle()
            self.move_gantry([self.joint_angles_for_raspberry_pi_camera_position[1], self.joint_angles_for_goal_position[2]])
        
        ## Move the Z Gantry back to the Raspberry Pi Camera location
        self.move_gantry([self.joint_angles_for_raspberry_pi_camera_position[1], self.joint_angles_for_raspberry_pi_camera_position[2]])

        ## Move Arm back to the Raspberry Pi Camera location
        self.move_arm([self.joint_angles_for_raspberry_pi_camera_position[3], self.joint_angles_for_raspberry_pi_camera_position[4], self.joint_angles_for_raspberry_pi_camera_position[5]])

    def determine_station_orientation_using_raspberry_pi_camera_2(self):
        self.desired_station_angle_in_degrees = (self.desired_station_angle_in_degrees + 180) % 360 - 180
        tbd_cv2 = TheBoatDoctorCV(self.actuator, self.station, self.station_orientation)
        ## Get the current angle of the station
        self.current_station_angle_in_degrees = float(tbd_cv2.get_station_info_pi())
        # num_tries = 1
        # while(self.current_station_angle_in_degrees == 10000 and num_tries < 3):
        #     #tbd_cv2 = TheBoatDoctorCV(self.actuator)
        #     num_tries = num_tries + 1
        #     self.current_station_angle_in_degrees = float(tbd_cv2.get_station_info_pi())

        if(self.current_station_angle_in_degrees == 10000):
            self.turn_on_leds()
            self.turn_off_leds()
            self.turn_on_leds()
            self.turn_off_leds()
            return False

        if(self.actuator == "V3"):
            if(abs(90 - abs(self.current_station_angle_in_degrees)) < self.v3_valve_threshold):
                self.current_station_angle_in_degrees = 90
            elif(abs(self.current_station_angle_in_degrees) < self.v3_valve_threshold):
                self.current_station_angle_in_degrees = 0

            if(self.station_orientation == "vertical"):
                self.current_station_angle_in_degrees = -self.current_station_angle_in_degrees

        print("Current Station Angle: " + str(self.current_station_angle_in_degrees))
        return True

    def get_current_station_angle_in_degrees(self):
        return self.current_station_angle_in_degrees

    def get_desired_breaker_position(self, breaker_command_index):
        return self.breaker_commands[2*breaker_command_index+1]

    def get_station_orientation(self):
        return self.station_orientation

    def get_num_breakers_to_actuate(self):
        return self.num_breakers_to_actuate

    def get_actuation_degree(self):
        return self.degree

    def set_actuation_degree(self, actuation_degree):
        self.degree = actuation_degree

    def verify_task_is_completed(self):
        if(self.actuator == "A" or self.actuator == "B"):
            self.num_breakers_to_actuate = 0
            self.breaker_commands = []

            for i in range(3):
                # Check if the breakers are already in their desired positions.
                if(self.current_breaker_positions[i] != self.desired_breaker_positions[i]):
                    # Check to see if the breakers are desired in the easy direction
                    if((self.desired_breaker_positions[i] == "D" and self.actuator == "A") or 
                       (self.desired_breaker_positions[i] == "U" and self.actuator == "B")):
                        self.num_breakers_to_actuate = self.num_breakers_to_actuate + 1
                        if(i == 0):
                            self.breaker_commands.append("B1")
                        elif(i == 1):
                            self.breaker_commands.append("B2")
                        elif(i == 2):
                            self.breaker_commands.append("B3")
                        self.breaker_commands.append(self.desired_breaker_positions[i])

            if(self.num_breakers_to_actuate == 0):
                return True
            else:
                return False
        else: 
            print("Degree Error = " + str(abs(((self.desired_station_angle_in_degrees - self.current_station_angle_in_degrees + 180) % 360) - 180)))
            return (abs(((self.desired_station_angle_in_degrees - self.current_station_angle_in_degrees + 180) % 360) - 180) < self.task_completion_angle_threshold_in_degrees)