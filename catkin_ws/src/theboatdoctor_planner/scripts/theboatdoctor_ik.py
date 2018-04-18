#!/usr/bin/env python

import numpy as np
import math

# Helper functions
def m_to_in(n):
    return n*39.3701

def in_to_m(n):
        return n*0.0254

def cam_to_ik(cam_coord):
    r_y_90 = np.array([ [0, 0, 1], [0, 1, 0], [-1, 0, 0] ])
    r_z_90 = np.array([ [0, -1, 0], [1, 0, 0], [0, 0, 1] ])
    return r_z_90.dot(r_y_90.dot(cam_coord))

class TheBoatDoctorIK:

    def __init__(self):
        # arm lengths in inches
        self.l1 = 6.5
        self.l2 = 7
        self.l3_vert = 2.5
        self.l3_horz = 4.5
        self.base_height = 6
        self.base_width = 2

        # Limits in inches
        self.x_gan_min = 0
        self.x_gan_max = 10
        self.z_gan_min = 0
        self.z_gan_max = 13
        self.z_arm_max_vert = 7
        self.z_arm_min_vert = -7
        self.z_arm_min_horz = -2.5
        self.z_arm_max_horz = 2.5

    def solve_ik(self, desired_end_effector_location, station_orientation):

        # coordinates from camera in mm
        x_cam = desired_end_effector_location[0]
        y_cam = desired_end_effector_location[1]
        z_cam = desired_end_effector_location[2]
        cam_coord = np.array([x_cam, y_cam, z_cam])

        if(station_orientation == "vertical"):
            return self.calc_ik_vert(cam_to_ik(m_to_in(cam_coord)))
        elif(station_orientation == "horizontal"):
            return self.calc_ik_horz(cam_to_ik(m_to_in(cam_coord)))
        else:
            print("Station orientation was not provided.")
            return [0,0,0,0,0,0]
            
    def calc_ik_horz(self, desired_end_effector_location):
        # position given is relative to base
        z_gan = self.z_gan_min
        z = desired_end_effector_location[2]
        x = desired_end_effector_location[0]
        while(z - z_gan > self.z_arm_max_horz):
            z_gan += 1
        while(z - z_gan < self.z_arm_min_horz):
            z_gan -=1
        if (z_gan > self.z_gan_max):
            z_gan = self.z_gan_max
        if (z_gan < self.z_gan_min):
            z_gan = self.z_gan_min
        z_arm = z - z_gan

        # calc angles based on position
        theta1 = math.asin((z_arm-self.l3_horz)/self.l2)
        theta2 = -math.pi/2 - theta1   

        # calc x values
        x_temp = self.l1 + self.l2 * math.cos(theta1)
        z_temp = self.l2 * math.sin(theta1) - self.l3_horz

        x_tot = x - x_temp
        if(x_tot < self.x_gan_min):
            x_gan = self.x_gan_min
        elif ( x_tot > self.x_gan_max):
            x_gan = self.x_gan_max
        else:
            x_gan = x - x_temp

        x_base = x - x_temp - x_gan
        z_gan = z - z_temp
        return np.array([0, in_to_m(x_gan), in_to_m(z_gan), theta1, theta2, 0])

    def calc_ik_vert(self, desired_end_effector_location):
        # position given is relative to base
        z_gan = self.z_gan_min
        z = desired_end_effector_location[2]
        x = desired_end_effector_location[0]
        while(z - z_gan > self.z_arm_max_vert):
            z_gan += 1
        while(z - z_gan < self.z_arm_min_vert):
            z_gan -=1
        if (z_gan > self.z_gan_max):
            z_gan = self.z_gan_max
        if (z_gan < self.z_gan_min):
            z_gan = self.z_gan_min
        z_arm = z - z_gan

        # calc angles based on position
        theta1 = math.asin(z_arm/self.l2)
        theta2 = -theta1

        # calc x values
        x_temp = self.l1 + self.l2 * math.cos(theta1) + self.l3_vert
        z_temp = self.l2 * math.sin(theta1)

        x_tot = x - x_temp
        if(x_tot < self.x_gan_min):
            x_gan = self.x_gan_min
        elif ( x_tot > self.x_gan_max):
            x_gan = self.x_gan_max
        else:
            x_gan = x - x_temp

        x_base = x - x_temp - x_gan
        z_gan = z - z_temp
        return np.array([0, in_to_m(x_gan), in_to_m(z_gan), theta1, theta2, 0])  
