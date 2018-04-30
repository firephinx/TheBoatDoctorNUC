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
        self.raspberry_pi_camera_vertical_station_v2_x_offset = -0.2
        self.raspberry_pi_camera_vertical_station_v2_z_offset = -0.06
        self.vertical_station_v2_goal_x_offset = -0.08
        self.vertical_station_v2_goal_z_offset = 0.03

        ## V3 Offsets
        self.raspberry_pi_camera_vertical_station_v3_x_offset = -0.2
        self.raspberry_pi_camera_vertical_station_v3_z_offset = -0.07
        self.vertical_station_v3_goal_x_offset = -0.069
        self.vertical_station_v3_goal_z_offset = -0.03

        ## Breaker Offsets
        self.raspberry_pi_camera_vertical_station_breaker_x_offset = -0.2
        self.raspberry_pi_camera_vertical_station_breaker_z_offset = -0.06
        self.vertical_station_breaker_goal_x_offset = -0.086
        self.vertical_station_breaker_goal_z_offset = 0.01

        self.breaker_1_x_offset = 0.0
        self.breaker_1_z_offset = 0.0
        self.breaker_2_x_offset = 0.0
        self.breaker_2_z_offset = 0.0
        self.breaker_3_x_offset = 0.0
        self.breaker_3_z_offset = 0.0

        ## Horizontal Station Offsets
        ## V1 Offsets
        self.raspberry_pi_camera_horizontal_station_v1_x_offset = -0.13
        self.raspberry_pi_camera_horizontal_station_v1_z_offset = 0.0
        self.horizontal_station_v1_goal_x_offset = -0.08
        self.horizontal_station_v1_goal_z_offset = 0.0

        ## V3 Offsets
        self.raspberry_pi_camera_horizontal_station_v3_x_offset = -0.06
        self.raspberry_pi_camera_horizontal_station_v3_z_offset = 0.2
        self.horizontal_station_v3_goal_x_offset = -0.086
        self.horizontal_station_v3_goal_z_offset = 0.01

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
            self.station_base_coords = [0.419, 0.734, 0.0]
        elif (self.station == "D"):
            self.station_base_coords = [0.419, 0.464, 0.0]
        elif (self.station == "E"):
            self.station_base_coords = [0.419, 0.414, 0.0]
        elif (self.station == "F"):
            self.station_base_coords = [0.419, 0.414, 0.0]
        elif (self.station == "G"):
            self.station_base_coords = [0.369, 0.414, 0.0]
        elif (self.station == "H"):
            self.station_base_coords = [0.749, 0.414, 0.0]
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
            self.station_turntable_degree = 60
        elif (self.station == "E"):
            #ensure the turntable is at 30 degrees
            self.station_turntable_degree = 30
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

    def set_station(self, station):
        self.station = station
        print("Station: " + self.station)

    def set_actuator(self, actuator):
        self.actuator = actuator
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
        (self.breaker_positions_in_3d_camera_coordinates, self.breaker_orientations) = self.tbd_cv.get_station_info_kinect()
        self.breaker_positions_in_3d_robot_coordinates.resize(3)
        for i in range(3):
            print("Breaker " + str(i) + " position in 3D camera coordinates: " + str(self.breaker_positions_in_3d_camera_coordinates[i]))
            print("Breaker " + str(i) + " orientation: " + self.breaker_orientations[i])
            self.breaker_positions_in_3d_robot_coordinates[i] = self.cam_to_ik(numpy.array(self.breaker_positions_in_3d_camera_coordinates[i]))
            print("Station object position in 3D robot coordinates: " + str(self.breaker_positions_in_3d_robot_coordinates))

    def determine_station_position_and_orientation_using_kinect(self):   
        (self.station_object_position_in_3d_camera_coordinates, self.station_orientation) = self.tbd_cv.get_station_info_kinect()

        print("Station object position in 3D camera coordinates: " + str(self.station_object_position_in_3d_camera_coordinates))
        print("Station orientation: " + self.station_orientation)
        self.station_object_position_in_3d_robot_coordinates = self.cam_to_ik(numpy.array(self.station_object_position_in_3d_camera_coordinates))
        print("Station object position in 3D robot coordinates: " + str(self.station_object_position_in_3d_robot_coordinates))

    ######### TODO: Make Better for all stations
    def generate_robot_trajectory_using_ik(self):
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
            
        elif(self.actuator == "V3"):
            self.raspberry_pi_camera_position = self.station_object_position_in_3d_robot_coordinates

            if(self.station_orientation == "vertical"):
                    self.raspberry_pi_camera_position[0] = self.raspberry_pi_camera_position[0] + self.raspberry_pi_camera_vertical_station_v3_x_offset
                    self.raspberry_pi_camera_position[2] = self.raspberry_pi_camera_position[2] + self.raspberry_pi_camera_vertical_station_v3_z_offset
            elif(self.station_orientation == "horizontal"):
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
            else:
                self.joint_angles_for_goal_position[1] = self.joint_angles_for_goal_position[1] - self.raspberry_pi_camera_horizontal_station_v3_x_offset + self.horizontal_station_v3_goal_x_offset
                self.joint_angles_for_goal_position[2] = self.joint_angles_for_goal_position[2] - self.raspberry_pi_camera_horizontal_station_v3_z_offset + self.horizontal_station_v3_goal_z_offset

        elif(actuator == "A" or actuator == "B"):
            
            self.joint_angles_for_goal_positions.resize(self.num_breakers_to_actuate)

            for i in range(self.num_breakers_to_actuate):
                breaker_index = 0
                breaker_x_offset = 0.0
                breaker_z_offset = 0.0
                if(self.actuations[2*i] == "B1"):
                    breaker_index = 0
                    breaker_x_offset = breaker_1_x_offset
                    breaker_z_offset = breaker_1_z_offset
                elif(self.actuations[2*i] == "B2"):
                    breaker_index = 1
                    breaker_x_offset = breaker_2_x_offset
                    breaker_z_offset = breaker_2_z_offset
                elif(self.actuations[2*i] == "B3"):
                    breaker_index = 2
                    breaker_x_offset = breaker_3_x_offset
                    breaker_z_offset = breaker_3_z_offset
                else:
                    print("Invalid breaker in mission file")
                    break
                breaker_goal_position = self.breaker_positions_in_3d_robot_coordinates[breaker_index]
                breaker_goal_position[0] = breaker_goal_position[0] + breaker_x_offset
                breaker_goal_position[2] = breaker_goal_position[2] + breaker_z_offset
                self.joint_angles_for_goal_positions[i] = self.tbd_ik.solve_ik(breaker_goal_position, "vertical")
               
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
        while(self.current_station_angle_in_degrees == 10000):
            self.current_station_angle_in_degrees = float(self.tbd_cv.get_station_info_pi())

        if(self.actuator == "V3"):
            if(abs(90 - abs(self.current_station_angle_in_degrees)) < self.v3_valve_threshold):
                self.current_station_angle_in_degrees = 90
            elif(abs(self.current_station_angle_in_degrees) < self.v3_valve_threshold):
                self.current_station_angle_in_degrees = 0

            if(self.station_orientation == "vertical"):
                self.current_station_angle_in_degrees = -self.current_station_angle_in_degrees

        print("Current Station Angle: " + str(self.current_station_angle_in_degrees))

    def determine_mission_goal(self):
        if(actuator == "A" or actuator == "B"):
            pass
        else:    
            if(self.actuator == "V1" or self.actuator == "V2"):
                self.direction = self.actuations[0][0]
                self.degree = float(self.actuations[0][1:])
                if(self.direction == "+"):
                    self.desired_station_angle_in_degrees = (self.current_station_angle_in_degrees + self.degree + 180) % 360 - 180 

                elif(self.direction == "-"):
                    self.desired_station_angle_in_degrees = (self.current_station_angle_in_degrees - self.degree + 180) % 360 - 180 

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
                    # Open
                    if(self.desired_station_position == "0"):
                        self.desired_station_angle_in_degrees = 0

                    # Closed
                    elif(self.desired_station_position == "1"):
                        self.desired_station_angle_in_degrees = 90

            print("Desired Station Angle: " + str(self.desired_station_angle_in_degrees))

    def update_waypoints_with_mission_goal(self):
        if(abs(self.desired_station_angle_in_degrees - self.current_station_angle_in_degrees) < self.angle_threshold_in_degrees):
            return
        else:
            self.desired_end_effector_angle_in_radians = (self.desired_station_angle_in_degrees - self.current_station_angle_in_degrees) * math.pi / 180
        
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

    def actuate_end_effector(self):
        self.move_arm([self.joint_angles_for_goal_position[3], self.joint_angles_for_goal_position[4], self.desired_end_effector_angle_in_radians])

    def turn_on_pump(self):
        self.tbd_controller.pump_switch("on")

    def turn_off_pump(self):
        self.tbd_controller.pump_switch("off")

    def return_to_raspberry_pi_camera_position(self):

        ## Home the arm and move the X Gantry back to the raspberry pi camera position
        if(self.station_orientation == "vertical"):
            self.move_gantry([self.joint_angles_for_raspberry_pi_camera_position[1], self.joint_angles_for_goal_position[2]])
            self.home_arm()
        else:
            self.home_arm()
            self.move_gantry([self.joint_angles_for_raspberry_pi_camera_position[1], self.joint_angles_for_goal_position[2]])
        
        ## Move the Z Gantry back to the Raspberry Pi Camera location
        self.move_gantry([self.joint_angles_for_raspberry_pi_camera_position[1], self.joint_angles_for_raspberry_pi_camera_position[2]])

        ## Move Arm back to the Raspberry Pi Camera location
        self.move_arm([self.joint_angles_for_raspberry_pi_camera_position[3], self.joint_angles_for_raspberry_pi_camera_position[4], self.joint_angles_for_raspberry_pi_camera_position[5]])

    def determine_station_orientation_using_raspberry_pi_camera_2(self):
        tbd_cv2 = TheBoatDoctorCV(self.actuator, self.station, self.station_orientation)
        ## Get the current angle of the station
        self.current_station_angle_in_degrees = float(tbd_cv2.get_station_info_pi())
        while(self.current_station_angle_in_degrees == 10000):
            #tbd_cv2 = TheBoatDoctorCV(self.actuator)
            self.current_station_angle_in_degrees = float(tbd_cv2.get_station_info_pi())

        if(self.actuator == "V3"):
            if(abs(90 - abs(self.current_station_angle_in_degrees)) < self.v3_valve_threshold):
                self.current_station_angle_in_degrees = 90
            elif(abs(self.current_station_angle_in_degrees) < self.v3_valve_threshold):
                self.current_station_angle_in_degrees = 0

            if(self.station_orientation == "vertical"):
                self.current_station_angle_in_degrees = -self.current_station_angle_in_degrees

        print("Current Station Angle: " + str(self.current_station_angle_in_degrees))

    def get_station_orientation(self):
        return self.station_orientation

    def verify_task_is_completed(self):
        if(actuator == "A" or actuator == "B"):
            self.num_breakers_in_task = len(self.actuations) / 2
            self.num_breakers_to_actuate = 0

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
                desired_breaker_position = self.actuations[2*i+1]
                # Check if the breaker is already in the desired position.
            return True
        else: 
            print("Degree Error = " + str(abs(self.desired_station_angle_in_degrees - self.current_station_angle_in_degrees) % 360))
            return (abs(self.desired_station_angle_in_degrees - self.current_station_angle_in_degrees) % 360  < self.task_completion_angle_threshold_in_degrees)