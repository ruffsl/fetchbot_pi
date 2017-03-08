#!/usr/bin/env python

# Core ROS imports come first.
import rospy

# Serial library
import serial
from threading import Thread
import threading

# ROS builtins
from sensor_msgs.msg import Imu

last_received = '' # Global for latest recived line

def receiving(ser):
    global last_received
    buffer_string = ''

    t = threading.currentThread() # Get current thread running function
    while getattr(t, "do_receive", True): # Watch for a stop signal
        inbuff = ser.inWaiting() # Wait for a buffer
        if inbuff > 0: # if we have somthing
            buffer_string = buffer_string + ser.read(inbuff) # add the buffer data to temp string
            if '\n' in buffer_string: # if we get a new line
                lines = buffer_string.split('\n') # Guaranteed to have at least 2 entries
                last_received = lines[-2] # move the second to last buffer split latest line
                #If the Arduino sends lots of empty lines, you'll lose the
                #last filled line, so you could make the above statement conditional
                #like so: if lines[-2]: last_received = lines[-2]
                buffer_string = lines[-1] # reset temp string to rest of buffer data

class SerialImuNode(object):
    '''Publishes IMU data from serial port'''

    def __init__(self):
        '''Initilize Serial IMU Node'''
        # Initilize PbD Session
        self._port_name = rospy.get_param('~port','/dev/rfcomm0')
        self._baud = rospy.get_param('~baud',9600)
        self._frame_id = rospy.get_param('~frame_id','imu_link')
        self._rate = rospy.get_param('~rate',10) # 10hz
        self.pub = rospy.Publisher('imu', Imu, queue_size=5)

        rospy.init_node('serial_imu_node')
        self.rate = rospy.Rate(self._rate)

        with serial.Serial(self._port_name, self._baud, timeout=1) as ser:
            rospy.loginfo("Connected to: " + ser.portstr)
            t = Thread(target=receiving, args=(ser,))
            t.start()
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
                    if len(values) == 10:
                        self.update_imu(values)
                        self.rate.sleep()
            stop_receiving = True
            t.do_receive = False
            t.join()

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
