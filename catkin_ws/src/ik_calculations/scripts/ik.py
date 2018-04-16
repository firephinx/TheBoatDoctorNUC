import numpy as np
import math

# arm lengths in inches
l1 = 6.5
l2 = 7
l3_vert = 2.5
l3_horz = 4.5
base_height = 6
base_width = 2

# Limits in inches
x_gan_min = 0
x_gan_max = 16
z_gan_min = 0.5
z_gan_max = 22
z_arm_max_vert = 7
z_arm_min_vert = -7
z_arm_min_horz = -2.5
z_arm_max_horz = 2.5

def mm_to_in(n):
  return n*0.0393701

def in_to_m(n):
  return n*0.0254

def cam_to_ik(cam_coord):
  r_y_90 = np.array([ [0, 0, 1], [0, 1, 0], [-1, 0, 0] ])
  r_z_90 = np.array([ [0, -1, 0], [1, 0, 0], [0, 0, 1] ])
  return r_z_90.dot(r_y_90.dot(cam_coord))

def calc_ik_horz(l3, coord, z_arm_min, z_arm_max):
  # position given is relative to base
  z_gan = z_gan_min
  z = coord[1]
  x = coord[0]
  while(z - z_gan > z_arm_max):
    z_gan += 1
  while(z - z_gan < z_arm_min):
    z_gan -=1
  if (z_gan > z_gan_max):
    z_gan = z_gan_max
  if (z_gan < z_gan_min):
    z_gan = z_gan_min
  z_arm = z - z_gan

  # calc angles based on position
  theta1 = math.asin((z_arm-l3)/l2)
  theta2 = -math.pi/2 - theta1   
  
  # calc x values
  x_temp = l1 + l2 * math.cos(theta1)
  z_temp = l2 * math.sin(theta1) - l3

  x_tot = x - x_temp
  if(x_tot < x_gan_min):
    x_gan = x_gan_min
  elif ( x_tot > x_gan_max):
    x_gan = x_gan_max
  else:
    x_gan = x - x_temp

  x_base = x - x_temp - x_gan
  z_gan = z - z_temp
  return np.array([in_to_m(x_base), in_to_m(x_gan), in_to_m(z_gan), theta1, theta2])

def calc_ik_vert(l3, coord, z_arm_min, z_arm_max):
  # position given is relative to base
  z_gan = z_gan_min
  z = coord[1]
  x = coord[0]
  while(z - z_gan > z_arm_max):
    z_gan += 1
  while(z - z_gan < z_arm_min):
    z_gan -=1
  if (z_gan > z_gan_max):
    z_gan = z_gan_max
  if (z_gan < z_gan_min):
    z_gan = z_gan_min
  z_arm = z - z_gan
  
  # calc angles based on position
  theta1 = math.asin(z_arm/l2)
  theta2 = -theta1

  # calc x values
  x_temp = l1 + l2 * math.cos(theta1) + l3
  z_temp = l2 * math.sin(theta1)

  x_tot = x - x_temp
  if(x_tot < x_gan_min):
    x_gan = x_gan_min
  elif ( x_tot > x_gan_max):
    x_gan = x_gan_max
  else:
    x_gan = x - x_temp

  x_base = x - x_temp - x_gan
  z_gan = z - z_temp
  return np.array([in_to_m(x_base), in_to_m(x_gan), in_to_m(z_gan), theta1, theta2])

def ik_pipeline():
  # coordinates from camera in mm
  x_cam = -500    ## Tong changed, was -300
  z_cam = 200    ## Tong changed, was  500
  cam_coord = np.array([x_cam, z_cam])
  
  # position of end effector
  vert = False
  ik_coord = cam_to_ik(mm_to_in(cam_coord))

  if (vert):
    ik_vals = calc_ik_vert(l3_vert, ik_coord, z_arm_min_vert, z_arm_max_vert)
  else:
    ik_vals = calc_ik_vert(l3_horz, ik_coord, z_arm_min_horz, z_arm_max_horz)
  return ik_vals    
