#!/usr/bin/env python
import rospy

import yaml
import numpy as np
import math

import sys
import time

import xbox

from RosInterface import ROSInterface

baseAccValue = 0
baseMagValue = 0
baseAngle = 0
ang_vel_rep_left = 0
ang_vel_rep_right = 0
period = 1.0/25.0
MeasState = [0, 0, 0, 0, 0]

timer = 0
square_index = 0
square_angles = [0, math.pi/2, math.pi, -math.pi/2]

from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise

class RobotControl(object):
    """
    Class used to interface with the rover. Gets sensor measurements through ROS subscribers,
    and transforms them into the 2D plane, and publishes velocity commands.
    """
    def __init__(self, pos_init, pos_goal, max_speed, max_omega):
        """
        Initialize the class
        """

        # Handles all the ROS related items
        self.ros_interface = ROSInterface()
        time.sleep(0.2)

        self.max_speed = max_speed
        self.max_omega = max_omega
        self.prev_lin_vel = 0.0
        self.prev_ang_vel = 0.0
        self.pos_init = pos_init
        self.pos_goal = pos_goal

        #self.joy = xbox.Joystick()         #Initialize joystick

        self.state = [pos_init[0], pos_init[1], 0.0]

        #Kalman initialization
        self.kalman_filter = KalmanFilter(dim_x=3, dim_z=3)
        self.kalman_filter.F = np.array([[1, 0, 0],[0, 1, 0],[0, 0, 1]]) # State Transition Matrix
        self.kalman_filter.H = np.array([[1, 0, 0],[0, 1, 0],[0, 0, 1]]) # Measurement function
        self.kalman_filter.P = np.array([[0, 0, 0],[0, 0, 0],[0, 0, 0]]) # covariance matrix

        self.kalman_filter.R = np.array([[0.035, 0, 0],[0, 0.035, 0],[0, 0, 0.001]]) # state uncertainty
        self.kalman_filter.Q = np.array([[0.005, 0, 0],[0, 0.005, 0],[0, 0, 0.008]]) # process uncertainty

        self.kalman_filter.x = np.array([self.state]).T
        #Uncomment as completed
        #self.diff_drive_controller = DiffDriveController(max_speed, max_omega)

    def KalmanUpdate(self):
    	self.kalman_filter.predict() #For the P matrix prediction

        self.kalman_filter.x = np.array([self.state]).T

        self.kalman_filter.update(np.array([MeasState[2:5]]).T) #MeasState[2:5] are the sensor data

        updated_state = [self.kalman_filter.x[0][0], self.kalman_filter.x[1][0], self.kalman_filter.x[2][0]]
        return updated_state

    def control(self):

        global ang_vel_rep_left, ang_vel_rep_right, period, baseAngle, baseMagValue, baseAccValue, MeasState
        global loop_ctr, AccList
        distances = self.ros_interface.get_distances()
        imu_meas = self.ros_interface.get_imu()

        #imu measurements
        MagValue = imu_meas[3:6]
        AccValue = [imu_meas[0] - baseAccValue[0], imu_meas[1] - baseAccValue[1]]

        #Skip acceleration noise
        if abs(AccValue[0]) < 0.15:
            AccValue[0] = 0.0

        if abs(AccValue[1]) < 0.15:
            AccValue[1] = 0.0

        Angle = -(math.atan2(MagValue[1],MagValue[0]) - baseAngle)

        #print([MagValue[0], MagValue[1]])

        #State estimation from measurements
        MeasState[0] = (self.prev_lin_vel + AccValue[0] * period) * math.cos(self.state[2]) #x #Velocity
        MeasState[1] = (self.prev_lin_vel + AccValue[0] * period) * math.sin(self.state[2]) #y

        MeasState[2] = self.state[0] + MeasState[0] * period #Position mhpws 8elei self.state anti gia measstate?
        MeasState[3] = self.state[1] + MeasState[1] * period
        MeasState[4] = Angle

        #Predict next state
        self.state[0] = self.state[0] + self.prev_lin_vel * period * math.cos(self.state[2]) # x
        self.state[1] = self.state[1] + self.prev_lin_vel * period * math.sin(self.state[2]) # y
        self.state[2] = self.state[2] + self.prev_ang_vel * period # theta

        #self.state[2] = Angle

        #Updated state, sensor fusion
        self.state = self.KalmanUpdate()
        #self.state[2] = (self.KalmanUpdate())[2]

        #Angle between -pi and pi
        if self.state[2] > math.pi:
        	self.state[2] -= 2 * math.pi
        elif self.state[2] < -math.pi:
        	self.state[2] += 2 * math.pi

        print(self.state[0], self.state[1], self.state[2] * 180 / math.pi)

        #Repultion fields from sensors
        ang_vel_rep_left = max(0.0, (self.max_omega/2 - 1.0*distances[0])* 3.0)
        ang_vel_rep_right = -max(0.0, (self.max_omega/2 - 1.0*distances[2])* 3.0)

        angle_from_goal = math.atan2(self.pos_goal[1] - self.state[1], self.pos_goal[0] - self.state[0])

        #Attraction field
        if abs(angle_from_goal - self.state[2]) < 0.1:
        	ang_vel_attr = 0
        else:
        	ang_vel_attr = 2.3 * (angle_from_goal - self.state[2])
        	ang_vel_attr = np.sign(ang_vel_attr) * min(0.7, abs(ang_vel_attr)) #angle_vel between -0.7 and 0.7

        #print(ang_vel_attr)
        lin_vel = min(self.max_speed, 5*(distances[1] - 0.16))

        #Linear velocity not below 0.1 m/s
        if lin_vel > 0:
            lin_vel = max(lin_vel, 0.1)
        else:
            lin_vel = min(lin_vel, -0.1)

        lin_vel = max(lin_vel, -self.max_speed)
        ang_vel = ang_vel_attr + ang_vel_rep_right + ang_vel_rep_left

        self.ros_interface.command_velocity(lin_vel, ang_vel)

        self.prev_lin_vel = lin_vel
        self.prev_ang_vel = ang_vel
        return

    def PerformSquare(self):
        global baseAngle, MeasState, square_angles, square_index, timer

        imu_meas = self.ros_interface.get_imu()

        #imu measurements
        MagValue = imu_meas[3:5]
        Angle = -(math.atan2(MagValue[1],MagValue[0]) - baseAngle)
        self.state[2] = Angle

        if self.state[2] > math.pi: #angle between -pi and pi
        	self.state[2] -= 2 * math.pi
        elif self.state[2] < -math.pi:
        	self.state[2] += 2 * math.pi

        if (square_index == 2) and (self.state[2] < 0):
            self.state[2] += 2 * math.pi

        print (self.state[2] * 180 / math.pi)

        if timer > 3:
            timer = -1
            square_index += 1

        angle_from_goal = square_angles[square_index]

        #Attraction field
        if abs(angle_from_goal - self.state[2]) < 0.06:
        	ang_vel_attr = 0
        else:
        	ang_vel_attr = 2.5 * (angle_from_goal - self.state[2])
        	ang_vel_attr = np.sign(ang_vel_attr) * min(1.5, abs(ang_vel_attr)) #angle_vel between -1.2 and 1.2

        self.ros_interface.command_velocity(0.2, ang_vel_attr)
        return

def main(args):
    global period,baseAccValue, baseMagValue, baseAngle, timer

    rospy.init_node('robot_control')

    # Setting Parameters
    pos_init = [0, 0]
    pos_goal = [2.5, 0.0]
    max_vel = 0.2
    max_omega = 1.4

    # Intialize the RobotControl object
    robotControl = RobotControl(pos_init, pos_goal, max_vel, max_omega)
    try:
        time.sleep(0.2)
        #Initial imu measurements
        init_meas = robotControl.ros_interface.get_imu()
        baseAccValue = init_meas[0:3]
        baseMagValue = init_meas[3:6]
        baseAngle = math.atan2(baseMagValue[1],baseMagValue[0])

        # Call process_measurements at 30Hz
        r = rospy.Rate(25)

        #While not within 10cm of the desired Position
        while (not rospy.is_shutdown()) and (math.sqrt((pos_goal[0] - robotControl.state[0])**2 + (pos_goal[1] - robotControl.state[1])**2) > 0.1):
            robotControl.control()
            timer = timer + period
            #robotControl.PerformSquare()
            r.sleep()

        # Done, stop robot
        robotControl.ros_interface.command_velocity(0,0)
    except:
        print
    finally:
        robotControl.ros_interface.GPIOCleanup()
        print("Cleaned!")

if __name__ == "__main__":
    try:
        main(sys.argv)
    except rospy.ROSInterruptException: pass
