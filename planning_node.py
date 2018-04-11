#!/usr/bin/env python

import math

#All measurements in meters and radians

def move_to_station(station, cur_base_pos):
    if (station == "A"):
        coords = [1.3462, 0.3048]
    elif (station == "B"):
        coords = [1.0922, 0.3048]
    elif (station == "C"):
        coords = [0.762, 0.3048]
    elif (station == "D"):
        coords = [0.4572, 0.3048]
    elif (station == "E"):
        coords = [0.2286, 0.3048]
    elif (station == "F"):
        coords = [0.3048, 0.2286]
    elif (station == "G"):
        coords = [0.3048, 0.4572]
    elif (station == "H"):
        coords = [0.3048, 0.762]
    else :
        coords = [0.2286, 0.2286]
    # Move the base to the coordiantes
    return coords

def move_turntable(station, cur_turntable_pos):
    cur_turntable_pos = 0
    if(station == "G" or station == "H"):
        # Ensure the turntable is at 90 degrees
        cur_turntable_pos = math.pi/2
    elif(station == "F"):
        # ensure the turntable is at -45 degrees
        cur_turntable_pos = -math.pi/4
    elif (station == "E"):
        #ensure the turntable is at 45 degrees
        cur_turntable_pos = math.pi/4
    else:
        #ensure the turntable is at 0
        cur_turntable_pos = 0
    return cur_turntable_pos

if __name__ == '__main__':
    # Define current posistions of the robot
    cur_turntable_pos = 0
    cur_base_pos = [0.2286, 0.2286]
    pathname = "all_stations.txt"
    f = open(pathname, "r")
    commands = f.readlines()
    for s in commands:
        s = s[0:len(s)-1]
        elements = s.split(' ')
        station = elements[0][0]
        cur_base_pos = move_to_station(station, cur_base_pos)
        cur_turntable_pos = move_turntable(station, cur_turntable_pos)
        actuator = elements[0][1:]
        actuations = elements[1:]
        if(actuator == "A" or actuator == "B"):
            # Breaker, assume breakers are always left to right in order
            directions = actuations[1::2]
        else:
            #valve
            direction = actuations[0][0]
            degree = actuations[0][1:]
    f.close()
        
