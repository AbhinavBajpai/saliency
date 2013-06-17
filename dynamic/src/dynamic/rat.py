#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import dynamic.rosdy


if __name__ == '__main__':
	rospy.init_node('rat', anonymous=True)
	rat = dyn.dynamicController()
	"""
	var = raw_input("Enter something: ")
	rat.join()
	var = raw_input("Enter something: ")
	rat.unjoin()
	"""
	rat.findCaps()
	#rat.join('hjk')
	#rospy.sleep(10)
	#rat.unjoin()
	#rospy.sleep(5)
	#rat.join('fgh')

	#job = rat.pairing_nodes(rat.pub)
	#jot = job[0]
	#rat.talk_nodes(rat.pub,jot)
	#topic = 'cap_%s' %(rat.pub)
	#print rat.pick_up_cap_srv(topic)
	#topic = 'cap_%s' %(rat.sub)
	#print rat.pick_up_cap_srv(topic)

	xs = rat.publist[0]
	xl = rat.sublist[0]
