#!/usr/bin/env python

import rospy
import serial

from sensor_msgs.msg import Imu

# import sys, select, termios, tty

class SerialImuNode(object):
    '''Publishes IMU data from serial port'''

    def __init__(self):
        '''Initilize Serial IMU Node'''
        # Initilize PbD Session
        self._serialport = rospy.get_param("serialport"))
        self.pub = rospy.Publisher('imu', Imu, queue_size=10)
        with serial.Serial(self._serialport, 9600, timeout=1) as ser:
            line = ser.read()
            print(line)

    # def update_imu(arg):
    #     imu_msg = Imu()
    #     imu_msg.header.frame_id = "imu_msg"
    #     imu_msg.header.stamp  = arg.current_real
    #     imu_msg.header.orientation : {
    #         x : imu.readMAGx(),
    #         y : imu.readMAGy(),
    #         z : imu.readMAGz()
    #     }
    #     imu_msg.orientation_covariance : [0,0,0,0,0,0,0,0,0]
    #     imu_msg.angular_velocity : {
    #         x : imu.readGYRx(),
    #         y : imu.readGYRy(),
    #         z : imu.readGYRz()
    #     }
    #     imu_msg.angular_velocity_covariance  : [0,0,0,0,0,0,0,0,0],
    #     imu_msg.linear_acceleration : {
    #          x : imu.readACCx(),
    #          y : imu.readACCy(),
    #          z : imu.readACCz()
    #      }
    #     imu_msg.linear_acceleration_covariance  : [0,0,0,0,0,0,0,0,0]
    #     rospy.loginfo(imu_msg)
    #     self.pub.publish(imu_msg)


if __name__ == '__main__':
    try:
        SerialImuNode()
    except rospy.ROSInterruptException:
        pass
