from subprocess import check_output
import os 
import signal

pid=check_output(["pidof","python","/home/theboatdoctor-nuc/TheBoatDoctorNUC/catkin_ws/src/object_detection/scripts/client.py --type 3"])
os.kill(int(pid.split(" ")[1]), signal.SIGKILL)                                                     
