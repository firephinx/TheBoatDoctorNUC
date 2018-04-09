alias s='source ~/.bashrc'
alias catm='catkin_make'
alias catws='cd ~/catkin_ws'
alias sshpi='ssh theboatdoctor-pi@192.168.1.30'
alias startkinect='bash /home/theboatdoctor-nuc/Documents/TheBoatDoctorNUC/scripts/start_kinect_on_NUC.sh'
alias startpicam='bash /home/theboatdoctor-nuc/Documents/TheBoatDoctorNUC/scripts/start_raspicam_node_on_NUC.sh'
alias startrosserial='bash /home/theboatdoctor-nuc/Documents/TheBoatDoctorNUC/scripts/start_rosserial_node_on_NUC.sh'
alias ledon='rostopic pub /TheBoatDoctor/LED_Switch std_msgs/Bool true --once'
alias ledoff='rostopic pub /TheBoatDoctor/LED_Switch std_msgs/Bool false --once'
alias pumpon='rostopic pub /TheBoatDoctor/Pump_Switch std_msgs/Bool true --once'
alias pumpoff='rostopic pub /TheBoatDoctor/Pump_Switch std_msgs/Bool false --once'
alias stop='rostopic pub /TheBoatDoctor/Stop std_msgs/Empty --once'