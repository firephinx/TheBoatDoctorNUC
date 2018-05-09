import numpy as np 


# ### Show img ### 
# showImg=0;  ### 0 means not showing hsv img  
# imshow_tip=0;
# dataQ_kinect=5
# dataQ_pi=11

# ######daytime
# ####### Kinect params ########## 




# ### 1. Shuttle ####
# ### At normal position 
# shuttle_low_th=np.array([100,100,100]) ## day: 100 150 50
# shuttle_high_th=np.array([150,240,240]) ## day: 150 220 100
# ## shuttle white part hsv threshold
# shuttle_white_low_th=np.array([50,0,220])
# shuttle_white_high_th=np.array([100,50,255])

# ###  At Station F
# ## shuttle hsv threshold 
# shuttle_low_th_F=np.array([80,70,100]) ## day: 100 70 100
# shuttle_high_th_F=np.array([130,220,150]) ## day: 130 220 150
# ## shuttle white part hsv threshold
# shuttle_white_low_th_F=np.array([50,0,200])
# shuttle_white_high_th_F=np.array([100,50,255])




# ### 2. Orange valve 
# orange_low_th=np.array([0,130,130])    # 0,130,200
# orange_high_th=np.array([20,220,255])  # 20,200,255
# ### At Station F 
# orange_low_th_F=np.array([0,130,130])    # 0,130,200
# orange_high_th_F=np.array([20,220,255])  # 20,200,255





# ### 3. Spigot valve 
# ### At normal position 
# spigot_low_th=np.array([100,200,100]) ## 100 200 100
# spigot_high_th=np.array([200,255,230]) ## 150 255 200
# ### sub proc: white part 
# spigot_white_low_th=np.array([50,0,200])        ## day  [50,0,200] night [50,0,230]
# spigot_white_high_th=np.array([120,50,255])     ## day  [120,50,255]

# ### At station F 
# spigot_low_th_F=np.array([100,200,100]) ## 100 200 100
# spigot_high_th_F=np.array([200,255,230]) ## 150 255 200
# ### sub proc: white part 
# spigot_white_low_th_F=np.array([50,0,200])        ## day  [50,0,200] night [50,0,230]
# spigot_white_high_th_F=np.array([120,50,255])     ## day  [120,50,255]





# ### 4. Breaker 
# ### Breaker A (41) 
# breaker_A_low_th_up=np.array([150,90,100])   ## [0,100,100]
# breaker_A_high_th_up=np.array([200,200,200]) ## [50,200,200]
# breaker_A_low_th_down=np.array([130,50,190])
# breaker_A_high_th_down=np.array([200,150,255]) 

# breaker_A_low_special=np.array([0,90,100])
# breaker_A_high_special=np.array([50,200,200])

# ### blk box 
# breaker_A_blk_box_low_th=np.array([0,100,0])   ## [50,50,0]
# breaker_A_blk_box_high_th=np.array([100,150,40]) ## [120,150,90]
 

# ### Breaker B (42)
# breaker_B_low_th_up=np.array([0,120,150])
# breaker_B_high_th_up=np.array([50,170,200])
# breaker_B_low_special=np.array([150,30,200])
# breaker_B_high_special=np.array([200,130,255])

# breaker_B_low_th_down=np.array([0,150,110])  
# breaker_B_high_th_down=np.array([50,220,200])
# ### blk box 
# breaker_B_blk_box_low_th=np.array([50,50,0])      ##[50,50,0]
# breaker_B_blk_box_high_th=np.array([120,150,40])  ##[120,150,90]











# ########## Raspiberry PI cam ####################### 



# #### 1. shuttle valve 
# pi_shuttle_low_th=np.array([0,160,0])
# pi_shuttle_high_th=np.array([150,255,220])




# #### 2. Orange Valve 
# pi_orange_low_th=np.array([0,100,80])
# pi_orange_high_th=np.array([100,200,200])
# ## green tip 
# pi_orange_green_tip_low_th=np.array([30,30,30])
# pi_orange_green_tip_high_th=np.array([100,140,100])





# #### 3. Spigot valve 
# ### horizontal 
# pi_spigot_low_th_horizontal=np.array([90,190,50])
# pi_spigot_high_th_horizontal= np.array([150,255,140])
# ### green tip 
# pi_spigot_green_low_th_horizontal= np.array([30,80,50])
# pi_spigot_green_high_th_horizontal= np.array([80,170,150])
# ### vertical 
# pi_spigot_low_th_vertical= np.array([100,200,60])
# pi_spigot_high_th_vertical= np.array([150,255,140])
# ### green tip 
# pi_spigot_green_low_th_vertical=np.array([30,30,30])
# pi_spigot_green_high_th_vertical=np.array([100,140,100])














######nighttime ######

### Show img ### 
showImg=0;  ### 0 means not showing hsv img  
imshow_tip=0;
dataQ_kinect=5
max_track_kinect=31
dataQ_pi=11
max_track_pi=31


####### Kinect params ########## 




### 1. Shuttle ####
### At normal position 
shuttle_low_th=np.array([100,100,100]) ## day: 100 150 50
shuttle_high_th=np.array([150,240,240]) ## day: 150 220 100
## shuttle white part hsv threshold
shuttle_white_low_th=np.array([50,0,220])
shuttle_white_high_th=np.array([120,50,255])

###  At Station F
## shuttle hsv threshold 
shuttle_low_th_F=np.array([100,100,100]) ## day: 100 70 100
shuttle_high_th_F=np.array([150,240,240]) ## day: 130 220 150
## shuttle white part hsv threshold
shuttle_white_low_th_F=np.array([0,0,235])
shuttle_white_high_th_F=np.array([30,30,255])




### 2. Orange valve 
orange_low_th=np.array([0,130,130])    # 0,130,200
orange_high_th=np.array([20,220,255])  # 20,200,255
### At Station F 
orange_low_th_F=np.array([0,130,130])    # 0,130,200
orange_high_th_F=np.array([20,220,255])  # 20,200,255





### 3. Spigot valve 
### At normal position 
spigot_low_th=np.array([90,200,50]) ## 100 200 100
spigot_high_th=np.array([140,255,160]) ## 150 255 200
### sub proc: white part 
spigot_white_low_th=np.array([0,0,200])        ## day  [50,0,200] night [50,0,230]
spigot_white_high_th=np.array([120,50,255])     ## day  [120,50,255]

### At station F 
spigot_low_th_F=np.array([100,200,100]) ## 100 200 100
spigot_high_th_F=np.array([200,255,230]) ## 150 255 200
### sub proc: white part 
spigot_white_low_th_F=np.array([50,0,200])        ## day  [50,0,200] night [50,0,230]
spigot_white_high_th_F=np.array([120,50,255])     ## day  [120,50,255]





### 4. Breaker 
### Breaker A (41) 
breaker_A_low_th_up=np.array([0,90,150])   ## [0,100,100]
breaker_A_high_th_up=np.array([50,200,210]) ## [50,200,200]
breaker_A_low_th_down=np.array([0,150,120])
breaker_A_high_th_down=np.array([50,220,170]) 

breaker_A_low_special=np.array([150,90,150])
breaker_A_high_special=np.array([200,200,200])

### blk box 
breaker_A_blk_box_low_th=np.array([50,50,0])   ## [50,50,0]
breaker_A_blk_box_high_th=np.array([120,150,30]) ## [120,150,90]
 

### Breaker B (42)
breaker_B_low_th_up=np.array([0,120,150])
breaker_B_high_th_up=np.array([50,170,200])
breaker_B_low_special=np.array([150,90,190])
breaker_B_high_special=np.array([200,150,240])

breaker_B_low_th_down=np.array([0,150,120])  
breaker_B_high_th_down=np.array([50,220,200])
### blk box 
breaker_B_blk_box_low_th=np.array([50,50,0])      ##[50,50,0]
breaker_B_blk_box_high_th=np.array([120,150,30])  ##[120,150,90]











########## Raspiberry PI cam ####################### 



#### 1. shuttle valve 
pi_shuttle_low_th=np.array([0,160,0])
pi_shuttle_high_th=np.array([150,255,220])




#### 2. Orange Valve 
pi_orange_low_th=np.array([0,100,80])
pi_orange_high_th=np.array([100,200,200])
## green tip 
pi_orange_green_tip_low_th=np.array([30,30,30])
pi_orange_green_tip_high_th=np.array([100,140,120])





#### 3. Spigot valve 
### horizontal 
pi_spigot_low_th_horizontal=np.array([90,140,50])
pi_spigot_high_th_horizontal= np.array([150,240,160])
### green tip 
pi_spigot_green_low_th_horizontal= np.array([30,80,50])
pi_spigot_green_high_th_horizontal= np.array([80,170,150])
### vertical 
pi_spigot_low_th_vertical= np.array([100,160,70])
pi_spigot_high_th_vertical= np.array([150,255,170])
### green tip 
pi_spigot_green_low_th_vertical=np.array([30,30,120])
pi_spigot_green_high_th_vertical=np.array([100,140,200])
