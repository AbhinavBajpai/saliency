#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import dynamic.rosdy as rosdy

def talker():
	pub = rosdy.Publisher('Fear', String, 'String')
	i=0
	r = rospy.Rate(1)
	while not rospy.is_shutdown():
		pub.publish(String('here %i'%(i)))
		r.sleep()

if __name__ == '__main__':
    rospy.init_node('q_p_talk', anonymous=True)
    try:
        talker()
    except rospy.ROSInterruptException: pass
