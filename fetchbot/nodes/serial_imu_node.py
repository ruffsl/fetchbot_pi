#!/usr/bin/env python

# Core ROS imports come first.
import rospy

# Serial library
import serial

# ROS builtins
from sensor_msgs.msg import Imu

class SerialImuNode(object):
    '''Publishes IMU data from serial port'''

    def __init__(self):
        '''Initilize Serial IMU Node'''
        # Initilize PbD Session
        self._port_name = rospy.get_param('~port','/dev/rfcomm0')
        self._baud = int(rospy.get_param('~baud','9600'))
        self._frame_id = rospy.get_param('~frame_id','imu_link')
        self.pub = rospy.Publisher('imu', Imu, queue_size=10)

        rospy.init_node('serial_imu_node')

        self.rate = rospy.Rate(10) # 10hz

        with serial.Serial(self._port_name, self._baud, timeout=1) as ser:
            while not rospy.is_shutdown():
                try:
                    line = ser.readline()
                    # print("line: ", line)
                    values = map(float, line.split(','))
                    # print("values: ", values)
                    self.update_imu(values)
                except:
                    continue
                self.rate.sleep()

    def update_imu(self, values):
        imu_msg = Imu()
        imu_msg.header.frame_id = self._frame_id
        # print("values: ", values)
        imu_msg.header.stamp  = rospy.Time.now()
        imu_msg.linear_acceleration.x = values[1]
        imu_msg.linear_acceleration.y = values[2]
        imu_msg.linear_acceleration.z = values[3]
        # imu_msg.linear_acceleration_covariance = [0,0,0,0,0,0,0,0,0]

        imu_msg.angular_velocity.x = values[4]
        imu_msg.angular_velocity.y = values[5]
        imu_msg.angular_velocity.z = values[6]
        # imu_msg.angular_velocity_covariance = [0,0,0,0,0,0,0,0,0]

        imu_msg.orientation.x = values[7]
        imu_msg.orientation.y = values[8]
        imu_msg.orientation.z = values[9]
        # imu_msg.orientation_covariance = [0,0,0,0,0,0,0,0,0]
        # print("imu_msg: ", imu_msg)
        self.pub.publish(imu_msg)
        rospy.loginfo(imu_msg)


if __name__ == '__main__':
    try:
        SerialImuNode()
    except rospy.ROSInterruptException:
        pass
