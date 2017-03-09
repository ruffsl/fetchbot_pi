#!/usr/bin/env python

# Core ROS imports come first.
import rospy

# Serial library
import serial
from threading import Thread
import threading
import numpy as np

# import transformations

# ROS builtins
from actionlib import SimpleActionClient
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker

from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped

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

# def fromtwovectors(u, v):
#     cos_theta = np.dot(np.linalg.norm(u), np.linalg.norm(v));
#     half_cos = np.sqrt(0.5 * (1.0 + cos_theta));
#     half_sin = np.sqrt(0.5 * (1.0 - cos_theta));
#     vec3 w = normalize(cross(u, v));
#     return quat(half_cos,
#                 half_sin * w.x,
#                 half_sin * w.y,
#                 half_sin * w.z);

def fromtwovectors(u, v):
    w = np.cross(u, v)
    q = [np.dot(u, v), w[0], w[1], w[2]]
    q[0] += np.linalg.norm(q)
    norm = np.linalg.norm(q)
    return q / norm

class SerialImuNode(object):
    '''Publishes IMU data from serial port'''

    def __init__(self):
        '''Initilize Serial IMU Node'''
        # Initilize Node
        rospy.init_node('serial_imu_node')

        self._port_name = rospy.get_param('~port','/dev/rfcomm0')
        self._baud = rospy.get_param('~baud',9600)
        self._frame_id = rospy.get_param('~frame_id','imu_link')
        self._rate = rospy.get_param('~rate',10) # 10hz
        self._movebase_ns = rospy.get_param('~movebase_ns', 'move_base')
        self._fetched_thresh = rospy.get_param('~fetched_thresh',1000)
        self._fetched = False

        self.message_pub = rospy.Publisher('imu', Imu, queue_size=5)
        self.marker_pub = rospy.Publisher('imu/marker', Marker, queue_size=5)
        self.sac = SimpleActionClient(self._movebase_ns, MoveBaseAction)

        self._goal = MoveBaseGoal()
        self._goal.target_pose.header.frame_id = "map";
    	self._goal.target_pose.header.stamp = rospy.Time.now();
    	self._goal.target_pose.pose.position.x = 0;
    	self._goal.target_pose.pose.position.x = 0;
    	self._goal.target_pose.pose.orientation.w = 1.0;

        self.rate = rospy.Rate(self._rate)

        while not rospy.is_shutdown():
            with serial.Serial(self._port_name, self._baud, timeout=0.01) as ser:
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
                            self.update(values)
                            self.rate.sleep()
                    if not t.isAlive():
                        break
                stop_receiving = True
                t.do_receive = False
                t.join()

    def update(self, values):
        imu_msg = self.update_imu(values)
        self.update_marker(imu_msg)
        norm = np.linalg.norm(values[7:9])
        if (norm > self._fetched_thresh) and (not self._fetched):
            self._fetched = True
            self.sac.send_goal(self._goal)
        if (norm < self._fetched_thresh) and (self._fetched):
            self._fetched = False
            self.sac.cancel_all_goals()

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
        self.message_pub.publish(imu_msg)
        # rospy.loginfo(imu_msg)
        return imu_msg

    def update_marker(self, imu_msg):
        u = [.1,.1,.1]
        v = [imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z]
        q = fromtwovectors(u, v)
        marker = Marker()
        marker.header.frame_id = self._frame_id
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.scale.x = 1
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.pose.orientation.x = q[1]
        marker.pose.orientation.y = q[2]
        marker.pose.orientation.z = q[3]
        marker.pose.orientation.w = q[0]
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        self.marker_pub.publish(marker)


if __name__ == '__main__':
    try:
        SerialImuNode()
    except rospy.ROSInterruptException:
        pass
