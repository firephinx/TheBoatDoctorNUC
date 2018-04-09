import numpy as np
import math

# arm lengths
l1 = 6.5
l2 = 7
l3_vert = 2.5
l3_horz = 4.5
base_height = 6
base_width = 2

# Limits
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

def cam_to_ik(cam_coord):
  r_y_90 = np.array([ [0, 0, 1], [0, 1, 0], [-1, 0, 0] ])
  r_z_90 = np.array([ [0, -1, 0], [1, 0, 0], [0, 0, 1] ])
  return r_z_90.dot(r_y_90.dot(cam_coord))

def check_coord(world_coord, ik_coord):
  x_limit = 63
  return (world_coord[0] + ik_coord[0]) > x_limit

def rot_tt(coord):
  r_z_90 = np.array([ [0, -1, 0], [1, 0, 0], [0, 0, 1] ])
  return r_z_90.dot(coord)

def calc_ik_horz(l3, coord, z_arm_min, z_arm_max):
  # position given is relative to base
  y_base = coord[1];
  z_gan = z_gan_min
  z = coord[2]
  x = coord[0]
  while(z - z_gan > z_arm_max):
    z_gan += 1
  while(z - z_gan < z_arm_min):
    z_gan -=1
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
  return np.array([y_base, x_base, 0, x_gan, z_gan, theta1, theta2])

def calc_ik_vert(l3, coord, z_arm_min, z_arm_max):
  # position given is relative to base
  y_base = coord[1]; 
  z_gan = z_gan_min
  z = coord[2]
  x = coord[0]
  while(z - z_gan > z_arm_max):
    z_gan += 1
  while(z - z_gan < z_arm_min):
    z_gan -=1
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
  return np.array([y_base, x_base, 0, x_gan, z_gan, theta1, theta2])

############### Tong: Please add constrains (max and min avaliable distance) on X_motion and Z_motion
def ik_pipeline():
  # coordinates from camera in mm
  x_cam = -500    ## Tong changed, was -300 
  y_cam = -300     ## Tong changed, was -400 
  z_cam = 200    ## Tong changed, was  500
  cam_coord = np.array([x_cam, y_cam, z_cam])
  # current world coordinates in ik view
  x_world = 0
  y_world = 0
  z_world = 0
  world_coord = np.array([x_world, y_world, z_world])
  print(world_coord[1])
  # store position of turn table
  tt_rot = False
  # position of end effector
  vert = False
  #print(cam_coord)
  ik_coord = cam_to_ik(mm_to_in(cam_coord))
  #print(ik_coord)
  if(check_coord(world_coord, ik_coord)):
    # rotate the turntable
    tt_rot = True
    ik_coord = rot_tt(ik_coord)
  else:
    tt_rot = False
  #print(tt_rot)
  #print(ik_coord)
  if (vert):
    ik_vals = calc_ik_vert(l3_vert, ik_coord, z_arm_min_vert, z_arm_max_vert)
    #print(calc_ik_vert(l3_vert, ik_coord, z_arm_min_vert, z_arm_max_vert))
  else:
    ik_vals = calc_ik_vert(l3_horz, ik_coord, z_arm_min_horz, z_arm_max_horz)
    #print(calc_ik_horz(l3_horz, ik_coord, z_arm_min_horz, z_arm_max_horz))
  if(tt_rot):
    temp = -ik_vals[1]
    ik_vals[1] = ik_vals[0]
    ik_vals[0] = temp
    ik_vals[2] = np.pi/2
  return ik_vals    
