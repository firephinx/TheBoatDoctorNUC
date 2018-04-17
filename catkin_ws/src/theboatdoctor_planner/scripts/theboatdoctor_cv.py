#!/usr/bin/env python

import rospy

class TheBoatDoctorCV:

	def __init__(self):
		self.data = []

	def get_station_info(self, actuator):
		station_object_position_in_3d = [0,0,0,0,0,0,0,0,0]
		station_orientation = 'up'
		return (station_object_position_in_3d, station_orientation)