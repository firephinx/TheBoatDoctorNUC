import numpy as np 


### Show img ### 
showImg=1;  ### 0 means not showing hsv img  

####### Kinect params ########## 

dataQ_kinect=111111


### 1. Shuttle ####
### At normal position 
shuttle_low_th=np.array([100,100,100]) ## day: 100 150 50
shuttle_high_th=np.array([150,240,240]) ## day: 150 220 100
## shuttle white part hsv threshold
shuttle_white_low_th=np.array([50,0,220])
shuttle_white_high_high_th=np.array([100,50,255])

###  At Station F
## shuttle hsv threshold 
shuttle_low_th_F=np.array([80,70,100]) ## day: 100 70 100
shuttle_high_th_F=np.array([130,220,150]) ## day: 130 220 150
## shuttle white part hsv threshold
shuttle_white_low_th_F=np.array([50,0,200])
shuttle_white_high_th_F=np.array([100,50,255])




### 2. Orange valve 
orange_low_th=np.array([0,130,130])    # 0,130,200
orange_high_th=np.array([20,220,255])  # 20,200,255
### At Station F 
orange_low_th_F=np.array([0,130,130])    # 0,130,200
orange_high_th_F=np.array([20,220,255])  # 20,200,255





### 3. Spigot valve 
### At normal position 
spigot_low_th=np.array([100,200,100]) ## 100 200 100
spigot_high_th=np.array([200,255,230]) ## 150 255 200
### sub proc: white part 
spigot_white_low_th=np.array([50,0,200])        ## day  [50,0,200] night [50,0,230]
spigot_white_high_th=np.array([120,50,255])     ## day  [120,50,255]

### At station F 
spigot_low_th_F=np.array([100,200,100]) ## 100 200 100
spigot_high_th_F=np.array([200,255,230]) ## 150 255 200
### sub proc: white part 
spigot_white_low_th_F=np.array([50,0,200])        ## day  [50,0,200] night [50,0,230]
spigot_white_high_th_F=np.array([120,50,255])     ## day  [120,50,255]





### 4. Breaker 
### Breaker A (41) 
breaker_A_low_th_up=np.array([0,100,100])  
breaker_A_high_th_up=np.array([50,200,200])
breaker_A_low_th_down=np.array([130,50,190])
breaker_A_high_th_down=np.array([200,150,255]) 
### blk box 
breaker_A_blk_box_low_th=np.array([50,50,0])
breaker_A_blk_box_high_th=np.array([120,150,90])


### Breaker B (42)
breaker_B_low_th_up=np.array([0,80,110])  
breaker_B_high_th_up=np.array([50,220,255])
breaker_B_low_th_down=np.array([130,50,170])
breaker_B_high_th_down=np.array([190,110,255])
### blk box 
breaker_B_blk_box_low_th=np.array([50,50,0])
breaker_B_blk_box_high_th=np.array([120,150,90])











########## Raspiberry PI cam ####################### 

dataQ_pi=11

#### 1. shuttle valve 
pi_shuttle_low_th=np.array([0,160,0])
pi_shuttle_high_th=np.array([150,255,220])




#### 2. Orange Valve 
pi_orange_low_th=np.array([0,100,80])
pi_orange_high_th=np.array([100,200,200])
## green tip 
pi_orange_green_tip_low_th=np.array([30,30,30])
pi_orange_green_tip_high_th=np.array([100,140,100])





#### 3. Spigot valve 
### horizontal 
pi_spigot_low_th_horizontal=np.array([100,200,60])
pi_spigot_high_th_horizontal= np.array([150,255,140])
### green tip 
pi_spigot_green_low_th_horizontal= np.array([50,50,100])
pi_spigot_green_high_th_horizontal= np.array([120,120,150])
### vertical 
pi_spigot_low_th_vertical= np.array([100,200,60])
pi_spigot_high_th_vertical= np.array([150,255,140])
### green tip 
pi_spigot_green_low_th_vertical=np.array([30,30,30])
pi_spigot_green_high_th_vertical=np.array([100,140,100])
