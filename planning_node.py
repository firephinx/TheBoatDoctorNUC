#!/usr/bin/env python

def move_to_station(station):
    if (station == "A"):
        coords = [53, 12]
    elif (station == "B"):
        coords = [43, 12]
    elif (station == "C"):
        coords = [30, 12]
    elif (station == "D"):
        coords = [18, 12]
    elif (station == "E"):
        coords = [9, 12]
    elif (station == "F"):
        coords = [12, 9]
    elif (station == "G"):
        coords = [12, 18]
    elif (station == "H"):
        coords = [12, 30]
    else :
        coords = [9, 9]
    # Move the base to the coordiantes

def move_turntable(station):
    if(station == "G" or station == "H"):
        # Ensure the turntable is at 90 degrees
    elif(station == "F"):
        # ensure the turntable is at -45 degrees
    elif (station == "E"):
        #ensure the turntable is at 45 degrees
    else:
        #ensure the turntable is at 0

if __name__ == '__main__':
    pathname = "all_stations.txt"
    f = open(pathname, "r")
    commands = f.readlines()
    for s in commands:
        s = s[0:len(s)-1]
        elements = s.split(' ')
        station = elements[0][0]
        move_to_station(station)
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
        
