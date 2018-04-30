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

        # tbd_planner.move_to_station(station)

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

        tbd_planner.determine_mission_goal()

        tbd_planner.update_waypoints_with_mission_goal()

        if(tbd_planner.verify_task_is_completed()):
            print("Task is already completed.")
            continue

        tbd_planner.move_to_station_object()

        tbd_planner.turn_on_pump()

        tbd_planner.actuate_end_effector()

        tbd_planner.turn_off_pump()

        if(actuator == "A" or actuator == "B"):
            tbd_planner.position_arm_for_vision()
        else:
            tbd_planner.return_to_raspberry_pi_camera_position()

            tbd_planner.determine_station_orientation_using_raspberry_pi_camera_2()

        task_completed_flag = tbd_planner.verify_task_is_completed()

        num_attempts = 1

        while(task_completed_flag == False and num_attempts < 3):
            num_attempts = num_attempts + 1

            tbd_planner.update_waypoints_with_mission_goal()

            tbd_planner.move_to_station_object()

            tbd_planner.turn_on_pump()

            tbd_planner.actuate_end_effector()

            tbd_planner.turn_off_pump()

            tbd_planner.return_to_raspberry_pi_camera_position()

            tbd_planner.determine_station_orientation_using_raspberry_pi_camera_2()

            task_completed_flag = tbd_planner.verify_task_is_completed()

        if(num_attempts == 3):
            print("Attempted the task 3 times, but was unable to complete the task. Moving on to the next task.")
        else:
            print("Task is completed.")

    f.close()