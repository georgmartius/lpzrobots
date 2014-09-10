#!/usr/bin/env python

# A simple closed loop controller for use with the barrel from lpz
# $ python closed-loop.py

import time
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray

class LPZRos(object):
    def __init__(self):
        self.name = "lpzros"
        rospy.init_node(self.name)
        # pub=rospy.Publisher("/motors", Float64MultiArray, queue_size=1)
        self.pub_motors  = rospy.Publisher("/motors", Float64MultiArray)
        self.sub_sensor = rospy.Subscriber("/sensors", Float64MultiArray, self.cb_sensors)
        # pub=rospy.Publisher("/chatter", Float64MultiArray)
        self.msg=Float64MultiArray()

        # controller
        self.A = np.eye(2) * 10.1

    def cb_sensors(self, msg):
        self.msg.data = []
        s = msg.data
        m = np.tanh(np.dot(self.A, s) + np.ones_like(s) * 0.1)

        print "m =", m
        
        self.msg.data.append(m[0])
        self.msg.data.append(m[1])
        # print("sending msg", msg)
        self.pub_motors.publish(msg)
        # time.sleep(0.1)

if __name__ == "__main__":
    lpzros = LPZRos()
    rospy.spin()
    # while not rospy.shutdown():
