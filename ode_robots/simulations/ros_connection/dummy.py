import rospy
from std_msgs.msg import Float64MultiArray
rospy.init_node("georg")
pub=rospy.Publisher("/motors", Float64MultiArray, queue_size=1)
msg=Float64MultiArray()
msg.data=[1.0,-0.5]
pub.publish(msg)
