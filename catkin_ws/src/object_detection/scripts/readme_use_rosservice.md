### Section A: to open a terminal for calling rosservice inside python### 
1. import os 
2. os.system("gnome-terminal -e 'rosrun object_detection pi_cam_client.py --type 3'")
3. os.system("gnome-terminal -e 'rosrun object_detection kinect_client.py --type 3'")

**note** 3 is the actuator type from commision file, as follows:
type 1: shuttle valvue 
type 2: orange vale 
type 3: spigot valve 
type 4: breaker 



### Section B: to close the terminal inside python
1. from subprocess import check_output
2. import os 
3. import signal

4. pid=check_output(["pidof","python","/home/theboatdoctor-nuc/TheBoatDoctorNUC/catkin_ws/src/object_detection/scripts/pi_cam_client.py(kinect_client for kinect) --type 3"])  
5. os.kill(int(pid.split(" ")[1]), signal.SIGKILL)

**note** step 4 is to find process id for the terminal that is just opened. You only need to make sure that the argument of python script in step 4 is the same as step 2 of the previosu section (e.g --type 3) 

**note** closing the terminal may not be necessary. Beacause each time you call the client node the old client node will be killed automatically





### How to integrate into planning node ### 
## setup (init) ##
1. You will always have these four topics to subscribe (topic,type), please init subsriber for them first 
For Kinect: 
A. /kinect2/actuator_location",Float32MultiArray     ## This topic gives you the location of each actuator; if the target is breaker, it will also give you breaker status e.g (up,down,up)
B. /kinect2/error_msg",String   ## This topic gives you 1 if error presents. The error usually means that the camera can't find actuator or can't determine actuator states. If receiving the error, you should readjust robot positon or robot arm pose to let camera have a better view    

For Pi cam: 
A. /raspicam_node/actuator_status",String  ## it gives you degree of rotation for each actuator 
B. /raspicam_node/error_status",String   ## The meaning is the same as Kinect's error msg 

## integrate into planning node ##
For kinect 
1. Move robot to desired position
2. Once positioned, call kinect client to get actuator locations (and status if it is breaker). Pls follow Section A to finish this step 
3. Inside kinect rosservice, ceratin amount of images will be collected and analyzed. Once finished, you will receive actuator location from /kinect2/actuator_location  (and status if it is breaker) 
4. Once received data, close opened terminal. Pls follow Section B  
5. Reset data to None 

For raspicam: 
if not breaker: ## since we use kinect to process everything for breaker 
	1. Move pi cam to desired postion 
	2. Once positioned, call kinect client to get actuator status (degree of rotation). Pls follow Section A to finish this step  
	3. Inside kinect rosservice, ceratin amount of images will be collected and analyzed. Once finished, you will receive actuator status from /raspicam_node/actuator_status
	4. Once received data, close opened terminal. Pls follow Section B
	5. Reset data to None 


**note** 
I don't want to integrate error msg (feedback system) at this point but it will be a simple addition. We just need to add two more steps for kinect and raspicam (1. after step 2: if received error msg, move to desired position      2. at the end, reset error_msg to None).   
