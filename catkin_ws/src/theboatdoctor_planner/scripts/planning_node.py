#!/usr/bin/env python

import rospy
import math
import numpy
import argparse
from theboatdoctor_controller import TheBoatDoctorController
from theboatdoctor_ik import TheBoatDoctorIK
from theboatdoctor_cv import TheBoatDoctorCV

#All measurements in meters and radians
    
def get_station_base_coords(station):
    if (station == "A"):
        base_coords = [1.3462, 0.3048, 0.0]
    elif (station == "B"):
        base_coords = [1.0922, 0.3048, 0.0]
    elif (station == "C"):
        base_coords = [0.762, 0.3048, 0.0]
    elif (station == "D"):
        base_coords = [0.4572, 0.3048, 0.0]
    elif (station == "E"):
        base_coords = [0.3048, 0.3048, 0.0]
    elif (station == "F"):
        base_coords = [0.3048, 0.3048, 0.0]
    elif (station == "G"):
        base_coords = [0.3048, 0.4572, 0.0]
    elif (station == "H"):
        base_coords = [0.3048, 0.762, 0.0]
    else :
        base_coords = [0.3048, 0.3048, 0.0]
    # Move the base to the coordinates
    return base_coords

def get_station_turntable_theta(station):
    if(station == "G" or station == "H"):
        # Ensure the turntable is at 90 degrees
        turntable_theta = math.pi/2
    elif(station == "F"):
        # ensure the turntable is at 60 degrees
        turntable_theta = math.pi/6
    elif (station == "E"):
        #ensure the turntable is at 30 degrees
        turntable_theta = math.pi/3
    else:
        #ensure the turntable is at 0
        turntable_theta = 0
    return turntable_theta

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Planning node for The Boat Doctor.')
    parser.add_argument('missionfilepathname', help='specify the mission file path name')

    args = parser.parse_args()

    tbd_controller = TheBoatDoctorController()
    tbd_ik = TheBoatDoctorIK()
    tbd_cv = TheBoatDoctorCV()

    tbd_c.home_robot()

    f = open(args.missionfilepathname, "r")
    commands = f.readlines()
    for s in commands:
        s = s[0:len(s)-1]
        elements = s.split(' ')
        station = elements[0][0]
        if (station != "A" and station != "B" and station != "C" and station != "D" and station != "E" and station != "F" and station != "G" and station != "H"):
            print("Invalid station or task in mission file.")
            break

        # Move the robot base so that it is directly facing the station.
        print("Station: " + station)
        station_base_coords = get_station_base_coords(station)
        done_moving_robot_base_to_station_flag = tbd_controller.move_robot_base(station_base_coords)
        while(!done_moving_robot_base_to_station_flag):
            done_moving_robot_base_to_station_flag = tbd_controller.move_robot_base(station_base_coords)

        cur_base_pos = tbd_controller.get_current_position()
        print("Base Position: [" + str(cur_base_pos[0]) + ", " + str(cur_base_pos[1]) + "]")

        # Turn the turntable an appropriate amount to face the station.
        station_turntable_theta = get_station_turntable_theta(station)
        done_turning_turntable_to_station_flag = tbd_controller.turn_turntable(station_turntable_theta)
        while(!done_turning_turntable_to_station_flag):
            done_turning_turntable_to_station_flag = tbd_controller.turn_turntable(station_turntable_theta)

        cur_turntable_theta = tbd_controller.get_current_turntable_position()
        print("Turntable Position: " + str(cur_turntable_theta))

        actuator = elements[0][1:]
        actuations = elements[1:]
        if(actuator == "A" or actuator == "B"):
            # Breaker, assume breakers are always left to right in order
            directions = actuations[1::2]
        else:
            #valve
            direction = actuations[0][0]
            degree = actuations[0][1:]

        (station_object_position_in_3d, station_orientation) = tbd_cv.get_station_info(actuator)

        if(actuator == "A" or actuator == "B"):
            # Breakers
            goal_positions = numpy.array([station_object_position_in_3d[0:3],
                                          station_object_position_in_3d[3:6],
                                          station_object_position_in_3d[6:9]])

            intermediate_positions = numpy.array([station_object_position_in_3d[3:6],
                                                  station_object_position_in_3d[0:3],
                                                  station_object_position_in_3d[3:6],
                                                  station_object_position_in_3d[6:9]])

            intermediate_positions[0][0] = intermediate_positions[0][0] + 0.05
            intermediate_positions[0][2] = intermediate_positions[0][2] - 0.1
            intermediate_positions[1][2] = intermediate_positions[1][2] - 0.075
            intermediate_positions[2][2] = intermediate_positions[2][2] - 0.075
            intermediate_positions[3][2] = intermediate_positions[3][2] - 0.075

            joint_angles_for_goal_positions = numpy.zeros(3,6)
            joint_angles_for_intermediate_positions = numpy.zeros(4,6)

            for i in xrange(3):
                joint_angles_for_goal_positions[i] = tbd_ik.solve_ik(goal_positions[i], station_orientation)

            for i in xrange(4):
                joint_angles_for_intermediate_positions[i] = tbd_ik.solve_ik(intermediate_positions[i], station_orientation)

            trajectory = numpy.zeros(11,6)
            trajectory[0] = joint_angles_for_intermediate_positions[0]
            trajectory[1] = joint_angles_for_intermediate_positions[1]
            trajectory[2] = joint_angles_for_goal_positions[0]
            trajectory[3] = joint_angles_for_intermediate_positions[1]
            trajectory[4] = joint_angles_for_intermediate_positions[2]
            trajectory[5] = joint_angles_for_goal_positions[1]
            trajectory[6] = joint_angles_for_intermediate_positions[2]
            trajectory[7] = joint_angles_for_intermediate_positions[3]
            trajectory[8] = joint_angles_for_goal_positions[2]
            trajectory[9] = joint_angles_for_intermediate_positions[3]
            trajectory[10] = joint_angles_for_intermediate_positions[0]

        else:
            intermediate_positions = [station_object_position_in_3d,
                                      station_object_position_in_3d]

            if(station_orientation == "vertical"):
                intermediate_positions[0][0] = intermediate_positions[0][0] + 0.05
                intermediate_positions[0][2] = intermediate_positions[0][2] - 0.1
                intermediate_positions[1][2] = intermediate_positions[1][2] - 0.075
            elif(station_orientation == "horizontal"):
                intermediate_positions[0][0] = intermediate_positions[0][0] - 0.1
                intermediate_positions[0][2] = intermediate_positions[0][2] - 0.05
                intermediate_positions[1][0] = intermediate_positions[1][0] - 0.075
            else:
                print("Station orientation was not provided.")
                break

            joint_angles_for_goal_position = numpy.zeros(1,6)
            joint_angles_for_intermediate_positions = numpy.zeros(2,6)

            joint_angles_for_goal_position = tbd_ik.solve_ik(station_object_position_in_3d, station_orientation)

            for i in xrange(2):
                joint_angles_for_intermediate_positions[i] = tbd_ik.solve_ik(intermediate_positions[i], station_orientation)

            trajectory = numpy.zeros(5,6)
            trajectory[0] = joint_angles_for_intermediate_positions[0]
            trajectory[1] = joint_angles_for_intermediate_positions[1]
            trajectory[2] = joint_angles_for_goal_position
            trajectory[3] = joint_angles_for_intermediate_positions[1]
            trajectory[4] = joint_angles_for_intermediate_positions[0]


    f.close()