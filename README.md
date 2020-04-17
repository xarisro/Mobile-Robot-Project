# Mobile-Robot-Project

## Python code for an autonomous differential-drive mobile robot, using raspberry pi. 

The robot was equipped with an imu and three ultrasonic distance sensors. The goal was to implement two control algorithms.
The first was to control the robot so that it reaches a 2D desired position, while avoiding any obstacles.
The second was to contol the robot so that it performs a square trajectory.

Among the files included, the file `RobotControl.py` is the main file contains the control algorithms for both of the 
previous tasks, while the file `RosInterface.py` is responsible for receiving and analyzing the signals from the imu and the 
distance sensors. 
