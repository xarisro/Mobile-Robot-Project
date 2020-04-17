#!/usr/bin/env python
import roslib
import numpy as np
import numpy.matlib
import sys
import rospy
import RTIMU
import Adafruit_LSM303

from std_msgs.msg import (
    Header,
)

from sensor_msgs.msg import (
    Imu,
)

from geometry_msgs.msg import (
    Vector3,
)

class ImuReader(object):
    def __init__(self):

        self.lsm303 = Adafruit_LSM303.LSM303()
        self._rate = 50
        self.mag_rate = 30

    def read_from_imu(self):
        accel, mag = self.lsm303.read()
        return accel,mag

    def fill_imu_msg(self,accel,mag):
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = 'imu'
        imu_msg.orientation_covariance[0] = -1
        imu_msg.angular_velocity = Vector3(mag[0],
                                           mag[1],
                                           mag[2])
        imu_msg.linear_acceleration = Vector3(-accel[1]/100,
                                              accel[0]/100,
                                              accel[2]/100)
        return imu_msg

def main(args):
    rospy.init_node('imu_rtimulib')
    imu_reader = ImuReader()
    imu_pub = rospy.Publisher('imu', Imu, queue_size=10)
    r = rospy.Rate(imu_reader._rate)
    
    while not rospy.is_shutdown():
        (accel, mag) = imu_reader.read_from_imu()
        imu_msg = imu_reader.fill_imu_msg(accel, mag)
        imu_pub.publish(imu_msg)
        r.sleep()

if __name__ == "__main__":
    try:
        main(sys.argv)
    except rospy.ROSInterruptException: pass
