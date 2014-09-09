#!/usr/bin/env python

import time
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray

rospy.init_node("georg")

# pub=rospy.Publisher("/motors", Float64MultiArray, queue_size=1)
pub=rospy.Publisher("/motors", Float64MultiArray)
# pub=rospy.Publisher("/chatter", Float64MultiArray)
msg=Float64MultiArray()

msg.data.append(1.0)
msg.data.append(-0.5)
# msg.data=[1.0,-0.5]
for i in range(100):
	msg.data = []
	msg.data.append(np.cos(2*np.pi*(i/10.)))
	msg.data.append(np.sin(2*np.pi*(i/10.)))
	print("sending msg", msg)
	pub.publish(msg)
	time.sleep(0.1)
	# rospy.spin()
