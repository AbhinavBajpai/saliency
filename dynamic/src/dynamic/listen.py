#!/usr/bin/env python

import rospy
import sys
from std_msgs.msg import String
import dynamic.rosdy as rosdy

def callback(data):
	rospy.loginfo(rospy.get_name()+"I heard %s",data.data)

if __name__ == '__main__':
	rospy.init_node('q_s_listen', anonymous=True)
	sub = rosdy.Subscriber(rospy.get_name(),String, 'String')
	sub.subscribe(callback)
	
