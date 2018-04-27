#!/usr/bin/env python

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
        self.vertical_station_v1_goal_x_offset = -0.086
        self.vertical_station_v1_goal_z_offset = 0.01

        ## V2 Offsets
        self.raspberry_pi_camera_vertical_station_v2_x_offset = -0.2
        self.raspberry_pi_camera_vertical_station_v2_z_offset = -0.06
        self.vertical_station_v2_goal_x_offset = -0.086
        self.vertical_station_v2_goal_z_offset = 0.01

        ## V3 Offsets
        self.raspberry_pi_camera_vertical_station_v3_x_offset = -0.2
        self.raspberry_pi_camera_vertical_station_v3_z_offset = -0.06
        self.vertical_station_v3_goal_x_offset = -0.086
        self.vertical_station_v3_goal_z_offset = 0.01

        ## Breaker Offsets
        self.raspberry_pi_camera_vertical_station_breaker_x_offset = -0.2
        self.raspberry_pi_camera_vertical_station_breaker_z_offset = -0.06
        self.vertical_station_breaker_goal_x_offset = -0.086
        self.vertical_station_breaker_goal_z_offset = 0.01

        ## Horizontal Station Offsets
        ## V1 Offsets
        self.raspberry_pi_camera_horizontal_station_v1_x_offset = -0.06
        self.raspberry_pi_camera_horizontal_station_v1_z_offset = 0.2
        self.horizontal_station_v1_goal_x_offset = -0.086
        self.horizontal_station_v1_goal_z_offset = 0.01

        ## V3 Offsets
        self.raspberry_pi_camera_horizontal_station_v3_x_offset = -0.06
        self.raspberry_pi_camera_horizontal_station_v3_z_offset = 0.2
        self.horizontal_station_v3_goal_x_offset = -0.086
        self.horizontal_station_v3_goal_z_offset = 0.01

        ## Thresholds
        self.angle_threshold = 3 * math.pi / 180
        self.task_completion_degree_threshold = 15

        self.tbd_controller = TheBoatDoctorController()
        self.tbd_ik = TheBoatDoctorIK()

        self.done_homing_flag = False
        self.done_moving_robot_base_flag = False
        self.done_moving_gantry_flag = False
        self.done_moving_arm_flag = False

        self.current_base_position = self.tbd_controller.get_current_base_position()

    def cam_to_ik(self, cam_coord):
        r_y_90 = numpy.array([ [0, 0, 1], [0, 1, 0], [-1, 0, 0] ])
        return r_y_90.dot(cam_coord)

    def get_station_base_coords(self, station):
        print("Station: " + station)
        if (station == "A"):
            station_base_coords = [0.419, 1.249, 0.0]
        elif (station == "B"):
            station_base_coords = [0.419, 0.999, 0.0]
        elif (station == "C"):
            station_base_coords = [0.419, 0.749, 0.0]
        elif (station == "D"):
            station_base_coords = [0.419, 0.499, 0.0]
        elif (station == "E"):
            station_base_coords = [0.419, 0.419, 0.0]
        elif (station == "F"):
            station_base_coords = [0.419, 0.419, 0.0]
        elif (station == "G"):
            station_base_coords = [0.499, 0.419, 0.0]
        elif (station == "H"):
            station_base_coords = [0.749, 0.419, 0.0]
        else :
            station_base_coords = [0.419, 0.419, 0.0]

        print("Desired Base Position: [" + str(station_base_coords[0]) + ", " + str(station_base_coords[1]) + ", " + str(station_base_coords[2]) + "]")

        return station_base_coords

    def get_station_turntable_degree(self, station):
        print("Station: " + station)
        if(station == "A" or station == "B" or station == "C" or station == "D"):
            #ensure the turntable is at 0 degrees
            station_turntable_degree = 0
        elif(station == "G" or station == "H"):
            # Ensure the turntable is at 90 degrees
            station_turntable_degree = 90
        elif(station == "F"):
            # ensure the turntable is at 60 degrees
            station_turntable_degree = 60
        elif (station == "E"):
            #ensure the turntable is at 30 degrees
            station_turntable_degree = 30
        else:
            #ensure the turntable is at 0 degrees
            station_turntable_degree = 0

        print("Desired Turntable Degree: [" + str(station_turntable_degree) + "]")
        
        return station_turntable_degree

    def reset_robot(self):
        self.tbd_controller.reset_robot()

    def home_robot(self):
        print("Homing robot")
        self.done_homing_flag = self.tbd_controller.home_robot()
        while(self.done_homing_flag != True):
            self.done_homing_flag = self.tbd_controller.home_robot()

    def home_arm(self):
        print("Homing arm")
        self.tbd_controller.home_arm()

    def print_current_base_position(self):
        self.current_base_position = self.tbd_controller.get_current_base_position()
        print("Current Base Position: [" + str(self.current_base_position[0]) + ", " + str(self.current_base_position[1]) + ", " + str(self.current_base_position[2]) + "]")

    def move_robot_base(self, desired_coords):
        # Move the robot base to the desired coords.
        self.done_moving_robot_base_flag = self.tbd_controller.move_robot_base(desired_coords)
        while(self.done_moving_robot_base_flag != True):
            self.done_moving_robot_base_flag = self.tbd_controller.move_robot_base(desired_coords)

    def move_to_station(self, station):
        self.print_current_base_position()

        # fetch the station base coords
        station_base_coords = self.get_station_base_coords(station)
    
        self.move_robot_base(station_base_coords)
        
        self.print_current_base_position()

    def turn_turntable(self, desired_turntable_degree):
        desired_turntable_theta = math.radians(desired_turntable_degree)
        self.done_turning_turntable_flag = self.tbd_controller.turn_turntable(desired_turntable_theta)
        while(self.done_turning_turntable_flag != True):
            self.done_turning_turntable_flag = self.tbd_controller.turn_turntable(desired_turntable_theta)

    def print_current_turntable_degree(self):
        self.current_turntable_degree = self.tbd_controller.get_current_turntable_degree()
        print("Current Turntable Degree: [" + str(self.current_turntable_degree) + "]")

    def turn_turntable_to_station(self, station):
        self.print_current_turntable_degree()

        # fetch the station turntable degree
        station_turntable_degree = self.get_station_turntable_degree(station)
        
        self.turn_turntable(station_turntable_degree)
        
        self.print_current_turntable_degree()

    def set_actuator(self, actuator):
        self.actuator = actuator

    def set_actuations(self, actuations):
        self.actuations = actuations

    def start_vision(self):
        self.tbd_cv = TheBoatDoctorCV(self.actuator)

    def position_arm_for_vision(self):
        self.tbd_controller.position_arm_for_vision()

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
            elif(station_orientation == "horizontal"):
                    self.raspberry_pi_camera_position[0] = self.raspberry_pi_camera_position[0] + self.raspberry_pi_camera_horizontal_station_v1_x_offset
                    self.raspberry_pi_camera_position[2] = self.raspberry_pi_camera_position[2] + self.raspberry_pi_camera_horizontal_station_v1_z_offset
            else:
                print("Station orientation was not provided.")

            self.joint_angles_for_raspberry_pi_camera_position = numpy.zeros([1,6])

            print("Raspberry Pi Camera Position: " + str(self.raspberry_pi_camera_position))

            self.joint_angles_for_raspberry_pi_camera_position = self.tbd_ik.solve_ik(self.raspberry_pi_camera_position, self.station_orientation)
            self.joint_angles_for_intermediate_position = numpy.copy(self.joint_angles_for_raspberry_pi_camera_position)
            self.joint_angles_for_goal_position = numpy.copy(self.joint_angles_for_raspberry_pi_camera_position)

            if(self.station_orientation == "vertical"):
                self.joint_angles_for_intermediate_position[2] = self.joint_angles_for_intermediate_position[2] - self.raspberry_pi_camera_vertical_station_v1_z_offset + self.vertical_station_v1_goal_z_offset
                self.joint_angles_for_goal_position[1] = self.joint_angles_for_goal_position[1] - self.raspberry_pi_camera_vertical_station_v1_x_offset + self.vertical_station_v1_goal_x_offset
                self.joint_angles_for_goal_position[2] = self.joint_angles_for_goal_position[2] - self.raspberry_pi_camera_vertical_station_v1_z_offset + self.vertical_station_v1_goal_z_offset
            else:
                self.joint_angles_for_intermediate_position[1] = self.joint_angles_for_intermediate_position[1] - self.raspberry_pi_camera_horizontal_station_v1_x_offset + self.horizontal_station_v1_goal_x_offset
                self.joint_angles_for_goal_position[1] = self.joint_angles_for_goal_position[1] - self.raspberry_pi_camera_horizontal_station_v1_x_offset + self.horizontal_station_v1_goal_x_offset
                self.joint_angles_for_goal_position[2] = self.joint_angles_for_goal_position[2] - self.raspberry_pi_camera_horizontal_station_v1_z_offset + self.horizontal_station_v1_goal_z_offset

            self.num_waypoints = 6
            self.trajectory = numpy.zeros([self.num_waypoints,6])
            self.trajectory[0] = self.joint_angles_for_raspberry_pi_camera_position
            self.trajectory[1] = self.joint_angles_for_intermediate_position
            self.trajectory[2] = self.joint_angles_for_goal_position
            self.trajectory[3] = self.joint_angles_for_goal_position
            self.trajectory[4] = self.joint_angles_for_intermediate_position
            self.trajectory[5] = self.joint_angles_for_raspberry_pi_camera_position
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
            self.joint_angles_for_intermediate_position = numpy.copy(self.joint_angles_for_raspberry_pi_camera_position)
            self.joint_angles_for_goal_position = numpy.copy(self.joint_angles_for_raspberry_pi_camera_position)

            self.joint_angles_for_intermediate_position[2] = self.joint_angles_for_intermediate_position[2] - self.raspberry_pi_camera_vertical_station_v2_z_offset + self.vertical_station_v2_goal_z_offset
            self.joint_angles_for_goal_position[1] = self.joint_angles_for_goal_position[1] - self.raspberry_pi_camera_vertical_station_v2_x_offset + self.vertical_station_v2_goal_x_offset
            self.joint_angles_for_goal_position[2] = self.joint_angles_for_goal_position[2] - self.raspberry_pi_camera_vertical_station_v2_z_offset + self.vertical_station_v2_goal_z_offset
            
            self.num_waypoints = 6
            self.trajectory = numpy.zeros([self.num_waypoints,6])
            self.trajectory[0] = self.joint_angles_for_raspberry_pi_camera_position
            self.trajectory[1] = self.joint_angles_for_intermediate_position
            self.trajectory[2] = self.joint_angles_for_goal_position
            self.trajectory[3] = self.joint_angles_for_goal_position
            self.trajectory[4] = self.joint_angles_for_intermediate_position
            self.trajectory[5] = self.joint_angles_for_raspberry_pi_camera_position
        elif(self.actuator == "V3"):
            self.raspberry_pi_camera_position = self.station_object_position_in_3d_robot_coordinates

            if(self.station_orientation == "vertical"):
                    self.raspberry_pi_camera_position[0] = self.raspberry_pi_camera_position[0] + self.raspberry_pi_camera_vertical_station_v3_x_offset
                    self.raspberry_pi_camera_position[2] = self.raspberry_pi_camera_position[2] + self.raspberry_pi_camera_vertical_station_v3_z_offset
            elif(station_orientation == "horizontal"):
                    self.raspberry_pi_camera_position[0] = self.raspberry_pi_camera_position[0] + self.raspberry_pi_camera_horizontal_station_v3_x_offset
                    self.raspberry_pi_camera_position[2] = self.raspberry_pi_camera_position[2] + self.raspberry_pi_camera_horizontal_station_v3_z_offset
            else:
                print("Station orientation was not provided.")

            self.joint_angles_for_raspberry_pi_camera_position = numpy.zeros([1,6])

            print("Raspberry Pi Camera Position: " + str(self.raspberry_pi_camera_position))

            self.joint_angles_for_raspberry_pi_camera_position = self.tbd_ik.solve_ik(self.raspberry_pi_camera_position, self.station_orientation)
            self.joint_angles_for_intermediate_position = numpy.copy(self.joint_angles_for_raspberry_pi_camera_position)
            self.joint_angles_for_goal_position = numpy.copy(self.joint_angles_for_raspberry_pi_camera_position)

            if(self.station_orientation == "vertical"):
                self.joint_angles_for_intermediate_position[2] = self.joint_angles_for_intermediate_position[2] - self.raspberry_pi_camera_vertical_station_v3_z_offset + self.vertical_station_v3_goal_z_offset
                self.joint_angles_for_goal_position[1] = self.joint_angles_for_goal_position[1] - self.raspberry_pi_camera_vertical_station_v3_x_offset + self.vertical_station_v3_goal_x_offset
                self.joint_angles_for_goal_position[2] = self.joint_angles_for_goal_position[2] - self.raspberry_pi_camera_vertical_station_v3_z_offset + self.vertical_station_v3_goal_z_offset
            else:
                self.joint_angles_for_intermediate_position[1] = self.joint_angles_for_intermediate_position[1] - self.raspberry_pi_camera_horizontal_station_v3_x_offset + self.horizontal_station_v3_goal_x_offset
                self.joint_angles_for_goal_position[1] = self.joint_angles_for_goal_position[1] - self.raspberry_pi_camera_horizontal_station_v3_x_offset + self.horizontal_station_v3_goal_x_offset
                self.joint_angles_for_goal_position[2] = self.joint_angles_for_goal_position[2] - self.raspberry_pi_camera_horizontal_station_v3_z_offset + self.horizontal_station_v3_goal_z_offset

            self.num_waypoints = 6
            self.trajectory = numpy.zeros([self.num_waypoints,6])
            self.trajectory[0] = self.joint_angles_for_raspberry_pi_camera_position
            self.trajectory[1] = self.joint_angles_for_intermediate_position
            self.trajectory[2] = self.joint_angles_for_goal_position
            self.trajectory[3] = self.joint_angles_for_goal_position
            self.trajectory[4] = self.joint_angles_for_intermediate_position
            self.trajectory[5] = self.joint_angles_for_raspberry_pi_camera_position


    def print_current_gantry_position(self):
        self.current_gantry_position = self.tbd_controller.get_current_gantry_position()
        print("Current Gantry Positions: " + str(self.current_gantry_position))

    def move_gantry(self, desired_positions):
        self.print_current_gantry_position()

        print("Desired Gantry Positions: " + str(desired_positions))
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

        ## Move Gantry to Raspberry Pi Camera location
        self.move_gantry([self.trajectory[0][1], self.trajectory[0][2]])

        ## Move Arm to Raspberry Pi Camera location
        self.move_arm([self.trajectory[0][3], self.trajectory[0][4], self.trajectory[0][5]])
            
    def determine_station_orientation_using_raspberry_pi_camera(self):
        ## Get the current angle of the station
        self.current_station_angle = float(self.tbd_cv.get_station_info_pi())
        while(self.current_station_angle == 10000):
            self.current_station_angle = float(self.tbd_cv.get_station_info_pi())
        print("Current Station Angle: " + str(self.current_station_angle))

    def update_waypoints_with_mission_goal(self):
        if(self.actuator == "V1" or self.actuator == "V2"):
            self.direction = self.actuations[0][0]
            self.degree = float(self.actuations[0][1:])
            if(self.direction == "+"):
                self.desired_station_angle = self.current_station_angle + (self.degree * math.pi / 180)

                if(abs(self.desired_station_angle - self.current_station_angle) < self.angle_threshold):
                    return
                else:
                    self.trajectory[3][5] = self.desired_station_angle - self.current_station_angle
                    self.trajectory[4][5] = self.desired_station_angle - self.current_station_angle

            elif(direction == "-"):
                self.desired_station_angle = self.current_station_angle - (degree * math.pi / 180)

                if(abs(self.desired_station_angle - self.current_station_angle) < self.angle_threshold):
                    return
                else:
                    self.trajectory[3][5] = self.desired_station_angle - self.current_station_angle
                    self.trajectory[4][5] = self.desired_station_angle - self.current_station_angle

        else:
            self.desired_station_position = actuations[0][0]

            if(station_orientation == "vertical"):
                # Open
                if(self.desired_station_position == 0):
                    self.desired_station_angle = -(math.pi / 2)

                    if(abs(self.desired_station_angle - self.current_station_angle) < self.angle_threshold):
                        return
                    else:
                        self.trajectory[3][5] = self.desired_station_angle - self.current_station_angle
                        self.trajectory[4][5] = self.desired_station_angle - self.current_station_angle

                # Closed
                elif(self.desired_station_position == 1):
                    self.desired_station_angle = 0

                    if(abs(self.desired_station_angle - self.current_station_angle) < self.angle_threshold):
                        return
                    else:
                        self.trajectory[3][5] = self.desired_station_angle - self.current_station_angle
                        self.trajectory[4][5] = self.desired_station_angle - self.current_station_angle

            elif(station_orientation == "horizontal"):
                # Open
                if(self.desired_station_position == 0):
                    self.desired_station_angle = 0

                    if(abs(self.desired_station_angle - self.current_station_angle) < self.angle_threshold):
                        return
                    else:
                        self.trajectory[3][5] = self.desired_station_angle - self.current_station_angle
                        self.trajectory[4][5] = self.desired_station_angle - self.current_station_angle

                # Closed
                elif(self.desired_station_position == 1):
                    self.desired_station_angle = (math.pi / 2)

                    if(abs(self.desired_station_angle - self.current_station_angle) < self.angle_threshold):
                        return
                    else:
                        self.trajectory[3][5] = self.desired_station_angle - self.current_station_angle
                        self.trajectory[4][5] = self.desired_station_angle - self.current_station_angle

    def move_to_station_object(self):
        self.home_arm()

        ## Move Arm and Gantry towards the station
        self.move_gantry([self.trajectory[1][1], self.trajectory[1][2]])
        self.move_arm([self.trajectory[1][3], self.trajectory[1][4], self.trajectory[1][5]])
        self.move_gantry([self.trajectory[2][1], self.trajectory[2][2]])

    def actuate_end_effector(self):
        self.move_arm([self.trajectory[3][3], self.trajectory[3][4], self.trajectory[3][5]])

    def turn_on_pump(self):
        self.tbd_controller.pump_switch("on")
        rospy.sleep(1)

    def turn_off_pump(self):
        self.tbd_controller.pump_switch("off")
        rospy.sleep(1)

    def return_to_raspberry_pi_camera_position(self):
        ## Move Gantry back to the raspberry pi camera position
        self.move_gantry([self.trajectory[4][1], self.trajectory[4][2]])
        self.home_arm()
        self.move_gantry([self.trajectory[5][1], self.trajectory[5][2]])
        self.move_arm([self.trajectory[5][3], self.trajectory[5][4], self.trajectory[5][5]])

    def determine_station_orientation_using_raspberry_pi_camera_2(self):
        tbd_cv2 = TheBoatDoctorCV(actuator)
        ## Get the current angle of the station
        self.current_station_angle = float(tbd_cv2.get_station_info_pi())
        while(self.current_station_angle == 10000):
            self.current_station_angle = float(self.tbd_cv2.get_station_info_pi())
        print("Current Station Angle: " + str(self.current_station_angle))

    def get_station_orientation(self):
        return self.station_orientation

    def verify_task_is_completed(self):

        return (self.actuator == "A" or self.actuator == "B" or (abs(self.desired_station_angle - self.current_station_angle) < self.task_completion_degree_threshold))