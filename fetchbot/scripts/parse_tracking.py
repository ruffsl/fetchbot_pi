#!/usr/bin/python
import sys
# import transformations
import rospy

import numpy as np

# import transformations

# ROS builtins
from actionlib import SimpleActionClient
from visualization_msgs.msg import Marker

from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped

def fromtwovectors(u, v):
    w = np.cross(u, v)
    q = [np.dot(u, v), w[0], w[1], w[2]]
    q[0] += np.linalg.norm(q)
    norm = np.linalg.norm(q)
    return q / norm

class SSLNode(object):
    '''Publishes SSL data'''

    def __init__(self):
        '''Initilize SSL'''
        # Initilize Node
        rospy.init_node('ssl_node')

        self._frame_id = rospy.get_param('~frame_id','imu_link')
        self._rate = rospy.get_param('~rate',10) # 10hz

        self.marker_pub = rospy.Publisher('/ssl/marker', Marker, queue_size=5)

        self.rate = rospy.Rate(self._rate)

        # with serial.Serial(self._port_name, self._baud, timeout=1) as ser:
        #     rospy.loginfo("Connected to: " + ser.portstr)
        #     t = Thread(target=receiving, args=(ser,))
        #     t.start()
        #     while not rospy.is_shutdown():
        #         values = None
        #         try:
        #             # line = ser.readline()
        #             line = last_received
        #             # print("line: ", line)
        #             values = map(float, line.split(','))
        #             # print("values: ", values)
        #         except:
        #             pass
        #         if values is not None:
        #             if len(values) == 10:
        #                 self.update(values)
        #                 self.rate.sleep()
        #     stop_receiving = True
        #     t.do_receive = False
        #     t.join()

    def process_file(self, fname):
        dater = parse_file(fname)

        for x in dater:
            self.update(x)
            self.rate.sleep()
    # values need to be a packed {x: float, y: float, z: float, t: float} dict
    def update(self, values):
        # imu_msg = self.update_imu(values)

        self.update_marker(values)
        # norm = np.linalg.norm(values[7:9])
        # if (norm > self._fetched_thresh) and (not self._fetched):
        #     self._fetched = True
        #     self.sac.send_goal(self._goal)
        # if (norm < self._fetched_thresh) and (self._fetched):
        #     self._fetched = False
        #     self.sac.cancel_all_goals()

    def update_marker(self, imu_msg):
        u = [.1,.1,.1]
        # print imu_msg
        v = [imu_msg['x'], imu_msg['y'], imu_msg['z']]
        q = fromtwovectors(u, v)
        marker = Marker()
        marker.header.frame_id = self._frame_id
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.scale.x = 1
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.x = q[1]
        marker.pose.orientation.y = q[2]
        marker.pose.orientation.z = q[3]
        marker.pose.orientation.w = q[0]
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        self.marker_pub.publish(marker)



# given an output filename, parse the file into a list of localization xyz datapoints
# result has type {x: float, y: float, z: float, t: int}
def parse_file(fname):
    dater = []
    t = 0
    with open(fname) as f:
        lneNum = 0
        for lne in f:
            lne = lne.split(' ')
            prevData = {"t": t, "x": 1, "y": 0, "z": 0}
            if lne[0] == "(0002):":

                prevData = {"t": t, "x": float(lne[1]), "y": float(lne[2]), "z": float(lne[3])}
                dater.append(prevData)
                #print "found it"
                t += 1

                lneNum = (lneNum+1) % 4
            elif lne[0] == "(0000):":
                if (lneNum % 4 == 0):
                    dater.append(prevData)
                    t+= 1 # TODO: timestamps are broken for prevdata
                lneNum = (lneNum+1) % 4


    return dater

if __name__ == "__main__":
    # ok -- read in data
    # print parse_file(sys.argv[1])
    nde = SSLNode()
    nde.process_file(sys.argv[1]) # assumes filename is passed as first arg
