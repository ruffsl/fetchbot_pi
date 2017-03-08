#!/usr/bin/env python

# Core ROS imports come first.
import rospy

# Serial library
import serial
from threading import Thread

# ROS builtins
from sensor_msgs.msg import Imu

last_received = ''

def receiving(ser):
    global last_received

    buffer_string = ''
    while True:
        inbuff = ser.inWaiting()
        if inbuff > 0:
            buffer_string = buffer_string + ser.read(inbuff)
            if '\n' in buffer_string:
                lines = buffer_string.split('\n') # Guaranteed to have at least 2 entries
                last_received = lines[-2]
                #If the Arduino sends lots of empty lines, you'll lose the
                #last filled line, so you could make the above statement conditional
                #like so: if lines[-2]: last_received = lines[-2]
                buffer_string = lines[-1]

class SerialImuNode(object):
    '''Publishes IMU data from serial port'''

    def __init__(self):
        '''Initilize Serial IMU Node'''
        # Initilize PbD Session
        self._port_name = rospy.get_param('~port','/dev/rfcomm0')
        self._baud = int(rospy.get_param('~baud','9600'))
        self._frame_id = rospy.get_param('~frame_id','imu_link')
        self.pub = rospy.Publisher('imu', Imu, queue_size=1)

        rospy.init_node('serial_imu_node')

        self.rate = rospy.Rate(10) # 10hz

        with serial.Serial(self._port_name, self._baud, timeout=1) as ser:
            rospy.loginfo("connected to: " + ser.portstr)
            serial_thread = Thread(target=receiving, args=(ser,)).start()
            while not rospy.is_shutdown():
                values = None
                try:
                    # line = ser.readline()
                    line = last_received
                    # print("line: ", line)
                    values = map(float, line.split(','))
                    # print("values: ", values)
                except:
                    pass
                if values is not None:
                    self.update_imu(values)
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
        # rospy.loginfo(imu_msg)


if __name__ == '__main__':
    try:
        SerialImuNode()
    except rospy.ROSInterruptException:
        pass
