### how to run ###
1. roslaunc myrobot_description robot.launch
2. in a sourced terminal, rosrun rviz rviz, add robot model 

## explanation of robot,launch ##
1. load urdf to rosparam server as robot_description 
2. set a fix tf relative to world 
3. load a ros pkg to listen to joint input and transfer to tf so that robot model in rviz can move (robot move using tf)
4. joint_state_publihser: allow user to input joint states to control robot 



## if joint_Stata_publisher can't run GUI ##

in terminal, rosrun joint_state_publisher joint_state_publisher _use_gui:=true

