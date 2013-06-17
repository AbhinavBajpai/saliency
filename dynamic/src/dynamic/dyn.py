"""
Setup classes for dynamic communication in Ros
@author: Phuicy
"""

import rospy
import sys
import struct
import select
import rospy
from rospy.impl.registration import get_topic_manager, set_topic_manager, Registration, get_registration_listeners
from rospy.impl.tcpros import get_tcpros_handler, DEFAULT_BUFF_SIZE
from rospy.service import *
import threading
import rosnode
from dynamic.srv import *
import random
from sensor_msgs.msg import Image

class pubCap(threading.Thread):
	def __init__(self, namep, data_classp, pubp, subp):
		threading.Thread.__init__(self)
		self.namep = namep
		self.data_classp = data_classp
		self.pubp = pubp
		self.subp = subp
		self.pub12 = rospy.Publisher('cap', Cap)
		self.cappa = Cap()
		self.cappa.name = namep
		self.cappa.msg = data_classp
		self.cappa.pub = pubp
		self.cappa.sub = subp

	def run(self):
		while not rospy.is_shutdown():
			self.pub12.publish(self.cappa)
			print 'published capp'
			rospy.sleep(2)
			
class srvCap(threading.Thread):
	def __init__(self, namep, data_classp, pubp, subp):
		threading.Thread.__init__(self)
		self.namep = namep
		self.data_classp = data_classp
		self.pubp = pubp
		self.subp = subp
		self.ret = CapResponse()
		self.ret.name = namep
		self.ret.msg = data_classp
		self.ret.pub = pubp
		self.ret.sub = subp

	def return_cap(self, data):
		return self.ret

	def run(self):
		hart = 'cap_%s' %(self.namep)
		s = rospy.Service(hart, Cap, self.return_cap)
		rospy.spin()

class param_cap():
	def __init__(self, namep, data_classp, pubp, subp):
		los = namep + '_msg'
		rospy.set_param(los, data_classp)
	
	def change(self, namep, data_classp, pubp, subp):
		rospy.set_param(namep + '_msg', data_classp)

class dynamicPublisher():
	'''
	Act as publisher from ros but without the necessity 
	to specify the topic prior to runtime
	'''
	def __init__(self, name, data_class, msg, rate=1):
		'''
		Act as publisher from ros but without the necessity 
		to specify the topic prior to runtime
		@param name: node name
		@type name: String
		@param data_class: ros msg type
		@type data_class: msg
		@param msg: ros msg type name
		@type msg: string
		@param rate: rate of publish (default = 1)
		@type rate: positive number
		'''
		self.data_class = data_class
		self.name = name
		self.topic = ''
		self.pub = None
		self.rate = rate
		self.set = False
		self.msg = msg
		param_cap(name,msg,1,0)
		#t = srvCap(name,msg,1,0)
		#t.start()

	
	def checkForTopic(self):
		'''
		Check for topic in parameter server, then runs change topic.
		Returns True if topic found, False if not.
		Sets self.topic and self.set
		'''
		if rospy.has_param(self.name):
			self.topic_test = rospy.get_param(self.name)
			if (self.topic != self.topic_test):
				self.topic = self.topic_test
				self.changeTopic()
				self.set = True
			return self.set
		else:
			self.topic = ''

			if (self.set):
				print self.pub.get_num_connections()
				self.pub.unregister()
				self.set = False
			else:
				self.set = False
			return self.set
				
	
	def changeTopic(self):
		'''
		change publisher based off self. topic and dataclass
		'''
		print 'change'
		if(self.set):
			self.pub.unregister()
			self.pub = rospy.Publisher(self.topic,self.data_class)
		else:
			self.pub = rospy.Publisher(self.topic,self.data_class)
		
		
	def publish(self,data):
		'''
		Publish from dynamic publisher
		@param data: data to publish
		@type data: msg designated
		'''
		self.pub.publish(data)
		
		
		
class dynamicSubscriber(object):
	'''
	Acts as a ROs subscriber, however without the necessity state topic prior to runtime
	'''
	def __init__(self, name, data_class, msg, rate=1, queue_size=None, buff_size=DEFAULT_BUFF_SIZE):
		'''
		Acts as a ROs subscriber, however without the necessity state topic prior to runtime
		@param name: Node name
		@type name: String
		@param data_class: ROs message
		@type data_class: msg
		@param msg: ROs message type name
		@type msg: string
		@param rate: Publish rate (default =1)
		@type rate: positive number
		@param queue_size: size of subscribing queue for callback (default = None)
		@type queue_size: positive number
		@param buff_size: Buffer size should be greater than queue
		@type buff_size: positive number
		'''
		self.topic = ''
		self.topic_test = ''
		self.set = False
		self.sub = None
		self.data_class = data_class
		self.rate = rate
		self.name = name
		self.change = False
		self.stop  = False
		self.msg = msg
		param_cap(self.name,msg,0,1)
		#t = srvCap(name,msg,0,1)
		#t.start()

	def checkForTopic(self):
		'''
		Checks for topic in parameter server. Returns True if topic is found, and the counter.
		'''
		if rospy.has_param(self.name):
			self.topic_test = rospy.get_param(self.name)
			if (self.topic != self.topic_test):
				self.topic = self.topic_test
				self.change = True
			else:
				self.change = False
			self.set = True
			return self.set
		else:
			if (self.set):
				self.set = False
				self.stop = True
			else:
				self.set = False
				self.stop = False
			return self.set


	def subscribe(self,callBack):
		'''
		Subscribe as in ROs. Loops constantly, and uses checkForTopics
		@param callBack: callback function to apply to incoming data
		@type callBack: pointer to function
		'''
		rospy.Rate(self.rate)
		while not rospy.is_shutdown():
			rospy.sleep(self.rate)
			test = self.checkForTopic()
			if (test):
				while not rospy.is_shutdown():
					self.sub = rospy.Subscriber(self.topic, self.data_class, callBack)
					rospy.sleep(self.rate)
					test = self.checkForTopic()
					if self.change :
						self.sub.unregister()
						self.sub = rospy.Subscriber(self.topic, self.data_class, callBack)
					if self.stop :
						self.sub.unregister()
						self.stop = False
						break
					rospy.spin()


class PickupCappabilities(threading.Thread):
	def callBack(data):
		print 'Got', data.name
		comms.append(data)
		

	def run(self):
		global comms
		comms = []
		sub = rospy.Subscriber('cap',Cap,self.callBack)
		

class node_cap(object):
	def __init__(self,name,msg,pub=0,sub=0,cap=None):
		self.name = name
		self.msg = msg
		if pub == None:
			self.pub = 0
		else:
			self.pub = pub
		if sub == None:
			self.sub = 0
		else:
			self.sub = sub
		if cap == None:
			self.cap = None
		else:
			self.cap = cap


class joinedob(object):
	topic = ''
	pub = []
	sub = []
	def __init__(self):
		self.topic = ''
		self.pub = []
		self.sub = []

	def addPub(self, pubn):
		if self.pub == []:
			self.pub = [pubn]
		else:
			self.pub.append(pubn)

	def addSub(self, subn):
		if self.sub == []:
			self.sub = [subn]
		else:
			self.sub.append(subn)


class dynamicController(object):
	'''
	Acts as controller to the dynamic publishers and subscribers.
	'''

	def __init__(self, namespace=None):
		self.cap = []
		self.namesp = namespace
		self.joined = []

	def findCaps(self):
		'''
		gathers nodes of names and get their parameters.
		'''
				
		
		if self.namesp == None:
			lis = rosnode.get_node_names()
			it = 0
		else:
			lis = rosnode.get_node_names(self.namesp)
			it = len(self.namesp) + 1
		for x in lis:
			if x[1+it] == 'q':
				tet = x + '_msg'
				if x[3+it]=='s':
					msgs = rospy.get_param(tet) 
					lots = node_cap(x,msgs,0,1)
					self.cap.append(lots)

				if x[3+it]=='p':
					msgs = rospy.get_param(tet) 
					lots = node_cap(x,msgs,1,0)
					self.cap.append(lots)
		



	def pick_up_cap_srv(self,topic):
		rospy.wait_for_service(topic)
		try:
			add_t = rospy.ServiceProxy(topic, Cap)
			resp1 = add_t(1)
			lots = node_cap(resp1.name,resp1.msg,resp1.pub,resp1.sub)
			self.cap.append(lots)
			return (resp1.name,resp1.msg,resp1.pub)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
	
	
	def pairing_nodes(self,check):
		danger = []
		for x in self.cap:
			if x.name == check:
				check_msg = x.msg
				check_pub = x.pub
				check_sub = x.sub
				break
		for x in self.cap:
			if x.msg == check_msg:
				if check_pub + x.pub == 1:
					 danger.append(x.name)
		return danger
			

	def msgToName(self, namen):
		'''
		returns the msg from a publisher or subscriber
		'''
		for i in self.cap:
			if i.name == namen:
				return i.msg
		return None;

	def subToMsg(self, msgn):
		cats = []
		for i in self.cap:
			if (i.msg == msgn and i.sub == 1):
				cats.append(i.name)
		if cats == []:
			return None
		else:
			return cats

	def pubToMsg(self, msgn):
		cats = []
		for i in self.cap:
			if (i.msg == msgn and i.pub == 1):
				cats.append(i.name)
		if cats == []:
			return None
		else:
			return cats

	def pubToName(self, namen):
		for i in self.cap:
			if i.name == namen:
				if i.pub == 1:
					return True
				else:
					return False
		return None

	def both(self):
		start = []
		for x in self.cap:
			start.append(x.name)
		if start == []:
			return None
		else:
			return start


	def pub(self):
		start = []
		for x in self.cap:
			if x.pub == 1:
				start.append(x.name)
		if start == []:
			return None
		else:
			return start

	def sub(self):
		start = []
		for x in self.cap:
			if x.sub == 1:
				start.append(x.name)
		if start == []:
			return None
		else:
			return start
		

	def joinable(self, namen):
		msd = self.msgToName(namen)
		rtr = self.pubToName(namen)
		if (msd!=None and rtr!=None):
			if rtr:
				return self.subToMsg(msd)
			else:
				return self.pubToMsg(msd)
		else:
			return None
	

	def join(self, subn, pubn, topic=None):
		if topic == None:
			topic = 'not%i' %(random.randint(1,999))
		rospy.set_param(pubn, topic)
		rospy.set_param(subn, topic)
		if self.hasTopic(topic):
			self.addPubToXTopic(topic,pubn)
			self.addSubToXTopic(topic,subn)
		else:
			art = joinedob()
			art.topic = topic
			art.addPub(pubn)
			art.addSub(subn)
			self.joined.append(art)
		rospy.sleep(1)

	def hasTopic(self, topicn):
		for x in self.joined:
			if topicn == x.topic:
				return True
		return False

	def removeNameJ(self, name):
		for x in self.joined:
			if name in x.pub:
				x.pub.remove(name)
			elif name in x.sub:
				x.sub.remove(name)

	def listNameJ(self,name):
		for x in self.joined:
			if name in x.pub:
				return x.sub
			elif name in x.sub:
				return x.pub
		return None

	def addPubToXTopic(self, topicn, pubn):
		for x in self.joined:
			if topicn == x.topic:
				x.pub.append(pubn)

	def addSubToXTopic(self, topicn, subn):
		for x in self.joined:
			if topicn == x.topic:
				x.pub.append(subn)
	
	def unjoin(self, subn, pubn):
		'''
		unjoin the subn to pubn
		'''
		rospy.delete_param(pubn)
		rospy.delete_param(subn)

	def disconnect(self, name):
		rospy.delete_param(name)
		self.removeNameJ(name)

	def talk_nodes(self, sub, pub):
		rand = 'not%i' %(random.randint(1,99999))
		rospy.set_param(pub, rand)
		rospy.set_param(sub, rand)			
		
		


