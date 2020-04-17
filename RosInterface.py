#!/usr/bin/env python
"""
ROS based interface for the Course Robotics Specialization Capstone Autonomous Rover.
Updated June 15 2016.
"""
# ROS imports
import roslib
import rospy

from std_msgs.msg import (
    Header,
)

from sensor_msgs.msg import Imu

from geometry_msgs.msg import (
    PoseArray,
    PoseStamped,
    Pose,
    Twist,
)

import RPi.GPIO as GPIO

import cv2
import yaml
import numpy as np
import time
import threading

import sys

# Extra utility functions
from utility import *

TRIG = [19, 16, 13]
ECHO = [4, 18, 21]


class ROSInterface(object):
    """
    Class used to interface with the rover. Gets sensor measurements through ROS subscribers,
    and transforms them into the 2D plane, and publishes velocity commands.
    """

    def __init__(self):
        global TRIG,ECHO

        # Internal variables
        self._no_imu=True
        self._imu = None
        self.distances = [4, 4, 4]

        #GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(TRIG,GPIO.OUT)
        GPIO.setup(ECHO,GPIO.IN)

        self.Measurement_Rate = rospy.Rate(60)

        #Updating distances in a paraller thread
        thread = threading.Thread(target=self.update_distances, args=())
        thread.daemon = True                            # Daemonize thread
        thread.start() # Start the execution

        # ROS publishers and subscribers
        self._pub = rospy.Publisher("/cmd_vel",Twist,queue_size=5)
        rospy.Subscriber("/imu", Imu, self._imu_callback)

    def _imu_callback(self, imu):
        """
        Callback function for IMU measurements
        """
        self._imu = [imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z, imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z]
        self._no_imu = False

    def get_imu(self):
        if self._no_imu:
            return None
        else:
            return self._imu

    #Finding Distance according to the sensor Datasheet
    def FindDistance(self,i):
        global TRIG,ECHO

        GPIO.output(TRIG[i], False)

        time.sleep(0.01)

        #Sending one us pulse signal to the triger pin
        GPIO.output(TRIG[i], True)
        time.sleep(0.00001)
        GPIO.output(TRIG[i], False)

        pulse_start_before = time.time()
        pulse_start = pulse_start_before

        #Pulse starts when echo = 1
        while (GPIO.input(ECHO[i])==0) and (pulse_start - pulse_start_before < 0.007):
          pulse_start = time.time()

        if pulse_start - pulse_start_before >= 0.007: #time out check
          return 4

        pulse_start_before = time.time()
        pulse_end = pulse_start_before

        #The echo has returned when echo becomes zero
        while GPIO.input(ECHO[i])==1 and (pulse_end - pulse_start_before < 0.014):
          pulse_end = time.time()

        pulse_duration = pulse_end - pulse_start

        distance = pulse_duration * 17150
        distance = round(distance, 2)
        return distance/100

    def update_distances(self):
        while not rospy.is_shutdown():
            self.distances = [self.FindDistance(0), self.FindDistance(1), self.FindDistance(2)]
            self.Measurement_Rate.sleep()

    def GPIOCleanup(self):
        GPIO.cleanup()

    def get_distances(self):
        return self.distances

    def command_velocity(self,vx,wz):
        """
        Commands the robot to move with linear velocity vx and angular
        velocity wz
        """
        twist=Twist()
        twist.linear.x = vx
        twist.angular.z = wz
        self._pub.publish(twist)
