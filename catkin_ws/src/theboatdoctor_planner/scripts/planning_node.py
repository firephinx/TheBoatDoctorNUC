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
    for task in commands:
        task = task.strip()
        elements = task.split(' ')
        station = elements[0][0]
        if (station != "A" and station != "B" and station != "C" and station != "D" and station != "E" and station != "F" and station != "G" and station != "H"):
            print("Invalid station or task in mission file.")
            break

        tbd_planner.reset_robot()

        tbd_planner.home_robot()

        tbd_planner.set_station(station)

        tbd_planner.move_to_station()

        tbd_planner.turn_turntable_to_station()

        # Substitute Deep Neural Network Station Recognition Here

        actuator = elements[0][1:]
        actuations = elements[1:]

        tbd_planner.set_actuator(actuator)
        tbd_planner.set_actuations(actuations)

        tbd_planner.start_vision()

        tbd_planner.position_arm_for_kinect_vision()

        num_attempts = 1

        if(actuator == "A" or actuator == "B"):
            kinect_vision_done_flag = tbd_planner.determine_breaker_positions_and_orientations_using_kinect()

            if(kinect_vision_done_flag == False):
                print("Skipping this station because the Kinect messed up.")
                continue

            tbd_planner.determine_mission_goal()

            if(tbd_planner.verify_task_is_completed()):
                print("Task is already completed.")
                continue

            tbd_planner.generate_robot_trajectory_using_ik()

            num_breakers_to_actuate = tbd_planner.get_num_breakers_to_actuate()

            for i in xrange(num_breakers_to_actuate):

                tbd_planner.update_waypoints_with_mission_goal()

                tbd_planner.move_to_breaker(i)

                tbd_planner.turn_on_pump()

                # if((tbd_planner.get_desired_breaker_position(i) == "U" and actuator == "A") or
                #    (tbd_planner.get_desired_breaker_position(i) == "D" and actuator == "B")):
                #     #tbd_planner.actuate_wrist(i)
                #     tbd_planner.move_gantry_to_actuate_breaker()
                # else:
                tbd_planner.actuate_wrist(i)

                tbd_planner.turn_off_pump()

                tbd_planner.home_arm_with_goal_end_effector_angle()

            tbd_planner.position_arm_for_kinect_vision()

            tbd_planner.turn_turntable_to_station()

            # Uncomment below to try the breakers multiple times
            # tbd_planner.start_vision()

            # tbd_planner.determine_breaker_positions_and_orientations_using_kinect()

            # task_completed_flag = tbd_planner.verify_task_is_completed()

            # while(task_completed_flag == False and num_attempts < 3):
            #     num_attempts = num_attempts + 1

            #     tbd_planner.generate_robot_trajectory_using_ik()

            #     tbd_planner.update_waypoints_with_mission_goal()

            #     num_breakers_to_actuate = tbd_planner.get_num_breakers_to_actuate()

            #     for i in xrange(num_breakers_to_actuate):

            #         tbd_planner.move_to_breaker(i)

            #         tbd_planner.turn_on_pump()

            #         tbd_planner.turn_on_pump()

            #         tbd_planner.actuate_wrist(i)

            #         tbd_planner.turn_off_pump()

            #     tbd_planner.position_arm_for_kinect_vision()

            #     tbd_planner.start_vision()

            #     tbd_planner.determine_breaker_positions_and_orientations_using_kinect()

            #     task_completed_flag = tbd_planner.verify_task_is_completed()

        elif(actuator == "V1" or actuator == "V2"):
            kinect_vision_done_flag = tbd_planner.determine_station_position_and_orientation_using_kinect()
            
            if(kinect_vision_done_flag == False):
                print("Skipping this station because the Kinect messed up.")
                continue

            if(station == "E" or station == "F"):
                print("Valve is in the corner. Skipping this station.")
                continue

            tbd_planner.generate_robot_trajectory_using_ik()

            tbd_planner.move_to_raspberry_pi_camera_position()

            if(tbd_planner.get_station_orientation() == "horizontal"):
                tbd_planner.turn_on_leds()

                raspberry_pi_camera_done_flag = tbd_planner.determine_station_orientation_using_raspberry_pi_camera()

                tbd_planner.turn_off_leds()
            else:
                raspberry_pi_camera_done_flag = tbd_planner.determine_station_orientation_using_raspberry_pi_camera()

            if(raspberry_pi_camera_done_flag == False):
                print("Skipping this station because the Raspberry Pi Camera messed up.")
                continue

            tbd_planner.determine_mission_goal()

            tbd_planner.update_waypoints_with_mission_goal_from_mission_file()

            tbd_planner.move_to_station_object()

            tbd_planner.turn_on_pump()
                
            tbd_planner.actuate_end_effector()

            tbd_planner.turn_off_pump()

            tbd_planner.return_to_raspberry_pi_camera_position()

            if(tbd_planner.get_station_orientation() == "horizontal"):
                tbd_planner.turn_on_leds()

                raspberry_pi_2_camera_done_flag = tbd_planner.determine_station_orientation_using_raspberry_pi_camera_2()

                tbd_planner.turn_off_leds()
            else:
                raspberry_pi_2_camera_done_flag = tbd_planner.determine_station_orientation_using_raspberry_pi_camera_2()

            if(raspberry_pi_2_camera_done_flag == False):
                print("Moving on from this station because the Raspberry Pi Camera messed up.")
                continue

            task_completed_flag = tbd_planner.verify_task_is_completed()

            while(task_completed_flag == False and num_attempts < 3):
                num_attempts = num_attempts + 1

                tbd_planner.update_waypoints_with_mission_goal()

                tbd_planner.move_to_station_object()

                tbd_planner.turn_on_pump()

                tbd_planner.actuate_end_effector()

                tbd_planner.turn_off_pump()

                if(num_attempts == 3):
                    tbd_planner.home_arm_with_goal_end_effector_angle()
                else:
                    tbd_planner.return_to_raspberry_pi_camera_position()

                    if(tbd_planner.get_station_orientation() == "horizontal"):
                        tbd_planner.turn_on_leds()

                        raspberry_pi_2_camera_done_flag = tbd_planner.determine_station_orientation_using_raspberry_pi_camera_2()

                        tbd_planner.turn_off_leds()
                    else:
                        raspberry_pi_2_camera_done_flag = tbd_planner.determine_station_orientation_using_raspberry_pi_camera_2()

                    if(raspberry_pi_2_camera_done_flag == False):
                        print("Moving on from this station because the Raspberry Pi Camera messed up.")
                        continue

                    task_completed_flag = tbd_planner.verify_task_is_completed()

        elif(actuator == "V3"):

            kinect_vision_done_flag = tbd_planner.determine_station_position_and_orientation_using_kinect()

            if(kinect_vision_done_flag == False):
                print("Skipping this station because the Kinect messed up.")
                continue

            if(station == "F"):
                print("Station is F, skipping this station.")
                continue

            if(station == "E" and tbd_planner.get_station_orientation() == "vertical"):
                print("Haven't tried this configuration yet. Skipping this station.")
                continue

            if(station != "E" and tbd_planner.get_station_orientation() == "horizontal"):
                tbd_planner.turn_turntable_to_v3_offset()

            tbd_planner.generate_robot_trajectory_using_ik()

            tbd_planner.move_to_raspberry_pi_camera_position()

            if(tbd_planner.get_station_orientation() == "horizontal"):
                tbd_planner.turn_on_leds()

                raspberry_pi_camera_done_flag = tbd_planner.determine_station_orientation_using_raspberry_pi_camera()

                tbd_planner.turn_off_leds()
            else:
                raspberry_pi_camera_done_flag = tbd_planner.determine_station_orientation_using_raspberry_pi_camera()

            if(raspberry_pi_camera_done_flag == False):
                print("Skipping this station because the Raspberry Pi Camera messed up.")
                continue

            tbd_planner.determine_mission_goal()

            if(tbd_planner.verify_task_is_completed()):
                print("Task is already completed.")
                continue

            tbd_planner.update_waypoints_with_mission_goal_from_mission_file()

            if(tbd_planner.get_station_orientation() == "horizontal"):

                tbd_planner.move_to_station_object()

                if(tbd_planner.get_current_station_angle_in_degrees() < 45):
                    valve_orientation = "off"
                else:
                    valve_orientation = "on"

                if(valve_orientation == "off"):    
                    tbd_planner.turn_on_pump()

                    tbd_planner.move_gantry_to_actuate_shuttlecock_valve()

                    tbd_planner.turn_off_pump()

                    # tbd_planner.move_to_shuttlecock_valve()

                    # tbd_planner.actuate_wrist(0)
                else:
                    tbd_planner.actuate_end_effector()

                    tbd_planner.move_gantry_to_actuate_shuttlecock_valve()

                    tbd_planner.home_arm_with_goal_end_effector_angle()

            else:

                tbd_planner.move_to_station_object()

                tbd_planner.turn_on_pump()
                    
                tbd_planner.actuate_end_effector()

                tbd_planner.turn_off_pump()

                tbd_planner.return_to_raspberry_pi_camera_position()

                raspberry_pi_2_camera_done_flag = tbd_planner.determine_station_orientation_using_raspberry_pi_camera_2()

                if(raspberry_pi_2_camera_done_flag == False):
                    print("Moving on from this station because the Raspberry Pi Camera messed up.")
                    continue

                task_completed_flag = tbd_planner.verify_task_is_completed()

                while(task_completed_flag == False and num_attempts < 3):
                    num_attempts = num_attempts + 1

                    tbd_planner.update_waypoints_with_mission_goal()

                    tbd_planner.move_to_station_object()

                    tbd_planner.turn_on_pump()

                    tbd_planner.actuate_end_effector()

                    tbd_planner.turn_off_pump()

                    if(num_attempts == 3):
                        tbd_planner.home_arm_with_goal_end_effector_angle()
                    else:
                        tbd_planner.return_to_raspberry_pi_camera_position()

                        raspberry_pi_2_camera_done_flag = tbd_planner.determine_station_orientation_using_raspberry_pi_camera_2()

                        if(raspberry_pi_2_camera_done_flag == False):
                            print("Moving on from this station because the Raspberry Pi Camera messed up.")
                            continue

                        task_completed_flag = tbd_planner.verify_task_is_completed()

        if(num_attempts == 3):
            print("Attempted the task 3 times. Moving on to the next task.")
        else:
            print("Task is completed.")

    f.close()