#!/usr/bin/env python

import rospy
import math
import numpy
import argparse
from theboatdoctor_planner import TheBoatDoctorPlanner

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Planning node for The Boat Doctor.')
    parser.add_argument('missionfilepathname', help='specify the mission file path name')

    args = parser.parse_args()

    tbd_planner = TheBoatDoctorPlanner()

    f = open(args.missionfilepathname, "r")
    commands = f.readlines()
    for s in commands:
        s = s[0:len(s)-1]
        elements = s.split(' ')
        station = elements[0][0]
        if (station != "A" and station != "B" and station != "C" and station != "D" and station != "E" and station != "F" and station != "G" and station != "H"):
            print("Invalid station or task in mission file.")
            break

        tbd_planner.reset_robot()

        tbd_planner.home_robot()

        tbd_planner.move_to_station(station)

        tbd_planner.turn_turntable_to_station(station)

        # Substitute Deep Neural Network Here

        actuator = elements[0][1:]
        actuations = elements[1:]

        tbd_planner.set_actuator(actuator)
        tbd_planner.set_actuations(actuations)

        tbd_planner.start_vision()

        tbd_planner.position_arm_for_vision()

        tbd_planner.determine_station_position_and_orientation_using_kinect()

        tbd_planner.generate_robot_trajectory_using_ik()

        tbd_planner.move_to_raspberry_pi_camera_position()

        tbd_planner.determine_station_orientation_using_raspberry_pi_camera()

        if(tbd_planner.verify_task_is_completed()):
            continue

        tbd_planner.update_waypoints_with_mission_goal()

        tbd_planner.move_to_station_object()

        tbd_planner.turn_on_pump()

        tbd_planner.move_to_goal_position()

        tbd_planner.turn_off_pump()

        tbd_planner.return_to_raspberry_pi_camera_position()

        tbd_planner.determine_station_orientation_using_raspberry_pi_camera_2()

        while(tbd_planner.verify_task_is_completed() != True):
            tbd_planner.update_waypoints_with_mission_goal()

            tbd_planner.move_to_station_object()

            tbd_planner.turn_on_pump()

            tbd_planner.actuate_end_effector()

            tbd_planner.turn_off_pump()

            tbd_planner.return_to_raspberry_pi_camera_position()

            tbd_planner.determine_station_orientation_using_raspberry_pi_camera_2()


        # if(actuator == "A" or actuator == "B"):
        #     # Breakers
        #     goal_positions = numpy.array([station_object_position_in_3d_robot_coordinates[0:3],
        #                                   station_object_position_in_3d_robot_coordinates[3:6],
        #                                   station_object_position_in_3d_robot_coordinates[6:9]])

        #     intermediate_positions = numpy.array([station_object_position_in_3d_robot_coordinates[3:6],
        #                                           station_object_position_in_3d_robot_coordinates[0:3],
        #                                           station_object_position_in_3d_robot_coordinates[3:6],
        #                                           station_object_position_in_3d_robot_coordinates[6:9]])

        #     intermediate_positions[0][0] = intermediate_positions[0][0] + 0.05
        #     intermediate_positions[0][2] = intermediate_positions[0][2] - 0.1
        #     intermediate_positions[1][2] = intermediate_positions[1][2] - 0.075
        #     intermediate_positions[2][2] = intermediate_positions[2][2] - 0.075
        #     intermediate_positions[3][2] = intermediate_positions[3][2] - 0.075

        #     joint_angles_for_goal_positions = numpy.zeros([3,6])
        #     joint_angles_for_intermediate_positions = numpy.zeros([4,6])

        #     for i in xrange(3):
        #         joint_angles_for_goal_positions[i] = tbd_ik.solve_ik(goal_positions[i], station_orientation)

        #     for i in xrange(4):
        #         joint_angles_for_intermediate_positions[i] = tbd_ik.solve_ik(intermediate_positions[i], station_orientation)

        #     trajectory = numpy.zeros([11,6])
        #     trajectory[0] = joint_angles_for_intermediate_positions[0]
        #     trajectory[1] = joint_angles_for_intermediate_positions[1]
        #     trajectory[2] = joint_angles_for_goal_positions[0]
        #     trajectory[3] = joint_angles_for_intermediate_positions[1]
        #     trajectory[4] = joint_angles_for_intermediate_positions[2]
        #     trajectory[5] = joint_angles_for_goal_positions[1]
        #     trajectory[6] = joint_angles_for_intermediate_positions[2]
        #     trajectory[7] = joint_angles_for_intermediate_positions[3]
        #     trajectory[8] = joint_angles_for_goal_positions[2]
        #     trajectory[9] = joint_angles_for_intermediate_positions[3]
        #     trajectory[10] = joint_angles_for_intermediate_positions[0]

        #     directions = actuations[1::2]

        # else:
        #     raspberry_pi_camera_position = station_object_position_in_3d_robot_coordinates

        #     if(station_orientation == "vertical"):
        #         raspberry_pi_camera_position[0] = raspberry_pi_camera_position[0] + raspberry_pi_camera_vertical_station_x_offset
        #         raspberry_pi_camera_position[2] = raspberry_pi_camera_position[2] + raspberry_pi_camera_vertical_station_z_offset
        #     elif(station_orientation == "horizontal"):
        #         raspberry_pi_camera_position[0] = raspberry_pi_camera_position[0] + raspberry_pi_camera_horizontal_station_x_offset
        #         raspberry_pi_camera_position[2] = raspberry_pi_camera_position[2] + raspberry_pi_camera_horizontal_station_z_offset
        #     else:
        #         print("Station orientation was not provided.")
        #         break

        #     joint_angles_for_raspberry_pi_camera_position = numpy.zeros([1,6])

        #     print(raspberry_pi_camera_position)

        #     joint_angles_for_raspberry_pi_camera_position = tbd_ik.solve_ik(raspberry_pi_camera_position, station_orientation)
        #     joint_angles_for_intermediate_position = numpy.copy(joint_angles_for_raspberry_pi_camera_position)
        #     joint_angles_for_goal_position = numpy.copy(joint_angles_for_raspberry_pi_camera_position)

        #     if(station_orientation == "vertical"):
        #         joint_angles_for_intermediate_position[2] = joint_angles_for_intermediate_position[2] - raspberry_pi_camera_vertical_station_z_offset + vertical_station_goal_z_offset
        #         joint_angles_for_goal_position[1] = joint_angles_for_goal_position[1] - raspberry_pi_camera_vertical_station_x_offset + vertical_station_goal_x_offset
        #         joint_angles_for_goal_position[2] = joint_angles_for_goal_position[2] - raspberry_pi_camera_vertical_station_z_offset + vertical_station_goal_z_offset
        #     else:
        #         joint_angles_for_intermediate_position[1] = joint_angles_for_intermediate_position[1] - raspberry_pi_camera_horizontal_station_x_offset + horizontal_station_goal_x_offset
        #         joint_angles_for_goal_position[1] = joint_angles_for_goal_position[1] - raspberry_pi_camera_horizontal_station_x_offset + horizontal_station_goal_x_offset
        #         joint_angles_for_goal_position[2] = joint_angles_for_goal_position[2] - raspberry_pi_camera_horizontal_station_z_offset + horizontal_station_goal_z_offset

        #     num_trajectories = 6
        #     trajectory = numpy.zeros([num_trajectories,6])
        #     trajectory[0] = joint_angles_for_raspberry_pi_camera_position
        #     trajectory[1] = joint_angles_for_intermediate_position
        #     trajectory[2] = joint_angles_for_goal_position
        #     trajectory[3] = joint_angles_for_goal_position
        #     trajectory[4] = joint_angles_for_intermediate_position
        #     trajectory[5] = joint_angles_for_raspberry_pi_camera_position

            
            # ## Get the current angle of the station
            # current_angle = float(tbd_cv.get_station_info_pi())
            # while(current_angle == 10000):
            #     current_angle = float(tbd_cv.get_station_info_pi())
            # print("Station angle = " + str(current_angle))

            # angle_threshold = 3 * math.pi / 180

            # if(actuator == "V1" or actuator == "V2"):
            #     direction = actuations[0][0]
            #     degree = float(actuations[0][1:])
            #     if(direction == "+"):
            #         desired_angle = current_angle + (degree * math.pi / 180)

            #         if(abs(desired_angle - current_angle) < angle_threshold):
            #             continue
            #         else:
            #             trajectory[3][5] = desired_angle - current_angle
            #             trajectory[4][5] = desired_angle - current_angle

            #     elif(direction == "-"):
            #         desired_angle = current_angle - (degree * math.pi / 180)

            #         if(abs(desired_angle - current_angle) < angle_threshold):
            #                 continue
            #         else:
            #             trajectory[3][5] = desired_angle - current_angle
            #             trajectory[4][5] = desired_angle - current_angle

            # else:
            #     desired_position = actuations[0][0]

            #     if(station_orientation == "vertical"):
            #         # Open
            #         if(desired_position == 0):
            #             desired_angle = -(math.pi / 2)

            #             if(abs(desired_angle - current_angle) < angle_threshold):
            #                 continue
            #             else:
            #                 trajectory[3][5] = desired_angle - current_angle
            #                 trajectory[4][5] = desired_angle - current_angle

            #         # Closed
            #         elif(desired_position == 1):
            #             desired_angle = 0

            #             if(abs(desired_angle - current_angle) < angle_threshold):
            #                 continue
            #             else:
            #                 trajectory[3][5] = desired_angle - current_angle
            #                 trajectory[4][5] = desired_angle - current_angle

            #     elif(station_orientation == "horizontal"):
            #         # Open
            #         if(desired_position == 0):
            #             desired_angle = 0

            #             if(abs(desired_angle - current_angle) < angle_threshold):
            #                 continue
            #             else:
            #                 trajectory[3][5] = desired_angle - current_angle
            #                 trajectory[4][5] = desired_angle - current_angle

            #         # Closed
            #         elif(desired_position == 1):
            #             desired_angle = (math.pi / 2)

            #             if(abs(desired_angle - current_angle) < angle_threshold):
            #                 continue
            #             else:
            #                 trajectory[3][5] = desired_angle - current_angle
            #                 trajectory[4][5] = desired_angle - current_angle

            # Move through the trajectories
            # for i in range(1,6):
            #     if(i == 1 or i == 5):
            #         tbd_controller.home_arm()
            #     print(i)
            #     print("X Gantry: " + str(trajectory[i][1]))
            #     print("Z Gantry: " + str(trajectory[i][2]))
            #     done_moving_gantry_flag = tbd_controller.move_gantry([trajectory[i][1],trajectory[i][2]])
            #     while(done_moving_gantry_flag != True):
            #         done_moving_gantry_flag = tbd_controller.move_gantry([trajectory[i][1],trajectory[i][2]]) 

            #     print("Elbow Angle: " + str(trajectory[i][3]))
            #     print("Wrist Angle: " + str(trajectory[i][4]))
            #     print("End Effector Angle: " + str(trajectory[i][5]))
            #     done_moving_arm_flag = tbd_controller.move_arm([trajectory[i][3],trajectory[i][4],trajectory[i][5]])
            #     while(done_moving_arm_flag != True):
            #         done_moving_arm_flag = tbd_controller.move_arm([trajectory[i][3],trajectory[i][4],trajectory[i][5]])

            #     if (i == 2):
            #         tbd_controller.pump_switch("on")
            #         rospy.sleep(1)
            #     elif (i == 3):
            #         tbd_controller.pump_switch("off")
            #         rospy.sleep(1)

    f.close()