#!/usr/bin/env python

import numpy as np
import math

# Helper functions
def m_to_in(n):
    return n*39.3701

def in_to_m(n):
        return n*0.0254

class TheBoatDoctorIK:

    def __init__(self):
        # arm lengths in inches
        self.l1 = 8.125
        self.l2 = 10.5
        self.l3_vert = 6.75
        self.l3_horz = 6.75
        self.base_height = 6
        self.base_width = 2

        # Limits in inches
        self.x_gan_min = 0
        self.x_gan_max = 8
        self.z_gan_min = 0
        self.z_gan_max = 13
        self.z_arm_max_vert = 10
        self.z_arm_min_vert = -10
        self.z_arm_min_horz = -18
        self.z_arm_max_horz = 3
        self.x_arm_max_horz = 18
        self.x_arm_min_horz = 8.2

    def solve_ik(self, desired_end_effector_location, station_orientation):

        # coordinates from camera in mm
        x_cam = desired_end_effector_location[0]
        y_cam = desired_end_effector_location[1]
        z_cam = desired_end_effector_location[2]
        cam_coord = np.array([x_cam, y_cam, z_cam])

        if(station_orientation == "vertical"):
            return self.calc_ik_vert(m_to_in(cam_coord))
        elif(station_orientation == "horizontal"):
            return self.calc_ik_horz(m_to_in(cam_coord))
        else:
            print("Station orientation was not provided.")
            return [0,0,0,0,0,0]
            
    def calc_ik_horz(self, desired_end_effector_location):
        # position given is relative to base
        z_gan = self.z_gan_min
        z = desired_end_effector_location[2]
        x = desired_end_effector_location[0]

        x_gan = self.x_gan_min
        while(x - x_gan > self.x_arm_max_horz):
            x_gan += 0.1
        while(x - x_gan < self.x_arm_min_horz):
            x_gan -=0.1
        if (x_gan > self.x_gan_max):
            x_gan = self.x_gan_max
        if (x_gan < self.x_gan_min):
            x_gan = self.x_gan_min
        x_arm = x - x_gan

        print("x_arm = " + str(x_arm))
        print("x_gan = " + str(x_gan))
        print("(x_arm - self.l1)/self.l2 = " + str((x_arm - self.l1)/self.l2))

        # calc angles based on position
        theta1 = math.acos((x_arm - self.l1)/self.l2)
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
        if(z_gan > self.z_gan_max and x_gan < self.x_gan_max):
        	x_gan = self.x_gan_max
        	x_arm = x - x_gan
        	theta1 = math.acos((x_arm - self.l1)/self.l2)
        	theta2 = -math.pi/2 - theta1
        	z_temp = self.l2 * math.sin(theta1) - self.l3_horz
        	x_base = x - x_arm - x_gan
        	z_gan = z - z_temp
        print("x_base = " + str(x_base))
        return np.array([0, in_to_m(x_gan), in_to_m(z_gan), -theta1, theta2, 0])

        # theta1 = 0.0
        # theta2 = -math.pi/2 - theta1

        # z = desired_end_effector_location[2]
        # x = desired_end_effector_location[0]

        # x_gan = x - self.l1 - self.l2
        # z_gan = z + self.l3_horz

        # x_gan = self.x_gan_min
        # while(x - x_gan > self.x_arm_max_horz):
        #     x_gan += 1
        # while(x - x_gan < self.x_arm_min_horz):
        #     x_gan -=1
        # if (x_gan > self.x_gan_max):
        #     x_gan = self.x_gan_max
        # if (x_gan < self.x_gan_min):
        #     x_gan = self.x_gan_min
        # x_arm = x - x_gan

        # print("x_arm = " + str(x_arm))
        # print("x_gan = " + str(x_gan))
        # print("(x_arm - self.l1)/self.l2 = " + str((x_arm - self.l1)/self.l2))

        # # calc angles based on position
           

        # # calc x values
        # x_temp = self.l1 + self.l2 * math.cos(theta1)
        # z_temp = self.l2 * math.sin(theta1) - self.l3_horz

        # x_tot = x - x_temp
        # if(x_tot < self.x_gan_min):
        #     x_gan = self.x_gan_min
        # elif ( x_tot > self.x_gan_max):
        #     x_gan = self.x_gan_max
        # else:
        #     x_gan = x - x_temp

        # x_base = x - x_temp - x_gan
        # z_gan = z - z_temp
        # return np.array([0, in_to_m(x_gan), in_to_m(z_gan), -theta1, theta2, 0])



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
        if(x_base > 0):
            z_gan = 3
            z_arm = z - z_gan

            print("z_arm = " + str(z_arm))
            print("z_gan = " + str(z_gan))
            print("z_arm/self.l2 = " + str(z_arm/self.l2))

            # calc angles based on position
            theta1 = math.asin(z_arm/self.l2)
            theta2 = -theta1
            x_temp = self.l1 + self.l2 * math.cos(theta1) + self.l3_vert
            x_gan = x - x_temp
        else:
            z_gan = z - z_temp
        
        return np.array([0, in_to_m(x_gan), in_to_m(z_gan), -theta1, theta2, 0])  
