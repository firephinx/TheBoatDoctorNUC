# TheBoatDoctorNUC

## Setup Instructions
1.  Follow this tutorial to create a usb for Ubuntu 16.04: https://tutorials.ubuntu.com/tutorial/tutorial-create-a-usb-stick-on-windows#0
2.  Boot from the usb and complete the Ubuntu setup
3.  Connect to the CMU-SECURE wireless network following instructions here: https://www.cmu.edu/computing/services/endpoint/network-access/wireless/how-to/connect.html (You can install the certificate by loading it on a usb drive or briefly turning on your phone's hotspot and allowing the NUC to connect to internet through the hotspot.)
4.  Run `sudo apt update && sudo apt upgrade`
5.  Install ROS Kinetic following instructions here: http://wiki.ros.org/kinetic/Installation/Ubuntu
6.  Install Teamviewer following instructions here: https://community.teamviewer.com/t5/Knowledge-Base/Installation-of-TeamViewer-on-a-Ubuntu-system/ta-p/45
7.  Install git by typing: `sudo apt install git`
8.  Install vim by typing: `sudo apt install vim`
9.	Install temperature sensor monitoring by typing: `sudo apt install lm-sensors` then `sudo sensors-detect` then `sudo service kmod start` and finally `sudo apt install psensor`
10.	Install CPU load monitoring by typing: `sudo apt install indicator-multiload`
11.	Install ros_control package by typing: `sudo apt-get install ros-kinetic-ros-control ros-kinetic-ros-controllers`
12.	Install hebiros following instructions here: http://wiki.ros.org/hebiros
13. 