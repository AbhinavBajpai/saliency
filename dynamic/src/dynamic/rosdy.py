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
import threading

from rosgraph.names import make_global_ns, make_caller_id

class sub_cap():
	def __init__(self, namep, UID, data_classp):
		los = namep + '_sub_msg_UID_' + UID
		rospy.set_param(los, data_classp)

class pub_cap():
	def __init__(self, name, UID, data_classp):
		los2 = name + '_pub_msg_UID_' + UID
		rospy.set_param(los2, data_classp)

	


class Publisher():
	'''
	Act as publisher from ros but without the necessity 
	to specify the topic prior to runtime
	'''
	def __init__(self, UID, data_class, msg, rate=1):
		'''
		Act as publisher from ros but without the necessity 
		to specify the topic prior to runtime
		@param UID: unique ID
		@type UID: String
		@param data_class: ros msg type
		@type data_class: msg
		@param msg: ros msg type name
		@type msg: string
		@param rate: rate of publish (default = 1)
		@type rate: positive number
		'''
		self.data_class = data_class
		self.name = rospy.get_name()
		self.topic = None
		self.pub = None
		self.rate = rate
		self.set = False
		self.msg = msg
		self.UID = UID
		pub_cap(self.name, self.UID, self.msg)
		#self.pub = rospy.Publisher(self.topic, self.data_class)
		rospy.on_shutdown(self.onShut)

	def onShut(self):
		if rospy.has_param(self.name + '_UID_' + self.UID):		
			rospy.delete_param(self.name + '_UID_' + self.UID)
		if rospy.has_param(self.name + '_pub_msg_UID_' + self.UID):		
			rospy.delete_param(self.name + '_pub_msg_UID_' + self.UID)

	def checkForTopic(self):
		'''
		Check for topic in parameter server, then runs change topic.
		Returns True if topic found, False if not.
		Sets self.topic and self.set
		'''
		if rospy.has_param(self.name + '_UID_' + self.UID):
			self.topic_test = rospy.get_param(self.name + '_UID_' + self.UID)
			if (self.topic != self.topic_test):
				self.topic = self.topic_test
				self.changeTopic()
				self.set = True
			return self.set
		else:
			self.topic = ''

			if (self.set):
				#print self.pub.get_num_connections()
				self.pub.unregister()
				self.set = False
			else:
				self.set = False
			return self.set
				
	
	def changeTopic(self):
		'''
		change publisher based off self. topic and dataclass
		'''
		if(self.set):
			#self.pub.unregister()
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
		
		
		
class Subscriber(object):
	'''
	Acts as a ROs subscriber, however without the necessity state topic prior to runtime
	'''
	def __init__(self
, UID, data_class, msg, rate=1, queue_size=None, buff_size=DEFAULT_BUFF_SIZE):
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
		self.name = rospy.get_name()
		self.change = False
		self.stop  = False
		self.msg = msg
		self.UID = UID
		sub_cap(self.name, self.UID ,msg)
		rospy.on_shutdown(self.onShut)
		self.lock = threading.Lock()

	def onShut(self):
		if rospy.has_param(self.name + '_UID_' + self.UID):		
			rospy.delete_param(self.name + '_UID_' + self.UID)
		if rospy.has_param(self.name + '_sub_msg_UID_' + self.UID):		
			rospy.delete_param(self.name + '_sub_msg_UID_' + self.UID)	


	def checkForTopic(self):
		'''
		Checks for topic in parameter server. Returns True if topic is found, and the counter.
		'''
		self.lock.acquire()
		try:
			if rospy.has_param(self.name + '_UID_' + self.UID):
				self.topic_test = rospy.get_param(self.name + '_UID_' + self.UID)
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
		finally:
		    self.lock.release()


	def subscribe(self,callBack):
		'''
		Subscribe as in ROs. Loops constantly, and uses checkForTopics
		@param callBack: callback function to apply to incoming data
		@type callBack: pointer to function
		'''
		try:
			while not rospy.is_shutdown():
				rospy.sleep(self.rate)
				test = self.checkForTopic()
				first = True 
				second = True
				if (test):
					while (not rospy.is_shutdown()) and second:

						self.sub = rospy.Subscriber(self.topic, self.data_class, callBack)
						first = True
						try:
							while (not rospy.core.is_shutdown()) and first and second:
								test = self.checkForTopic()
								if self.change :
									self.sub.unregister()
									#self.sub.close()
									self.sub = None
									first = False
								if self.stop :
									self.sub.unregister()
									#self.sub.close()
									self.sub = None
									self.stop = False
									first = False
									second = False
								rospy.rostime.wallsleep(0.5)
						except KeyboardInterrupt:
							logdebug("keyboard interrupt, shutting down")
							rospy.core.signal_shutdown('keyboard interrupt')



		except KeyboardInterrupt:
			logdebug("keyboard interrupt, shutting down")
			rospy.core.signal_shutdown('keyboard interrupt')

		

class pubList(object):
	def __init__(self,name,UID,msg):
		self.name = name
		self.msg = msg
		self.UID = UID
		self.fut = self.name + '_UID_' + self.UID
	
	def full(self):
		return self.fut

class subList(object):
	def __init__(self,name,UID,msg):
		self.name = name
		self.msg = msg
		self.UID = UID
		self.fut = self.name + '_UID_' + self.UID

	def full(self):
		return self.fut

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


def findNameUID(name):
	little = []
	paramList = rospy.get_param_names()
	for x in paramList:
		if name in x:
			little.append(x)
	if little == []:
		return None
	else:
		return little

class Controller(object):
	'''
	Acts as controller to the dynamic publishers and subscribers.
	'''

	def __init__(self, namespace=None):
		self.sublist = []
		self.publist = []
		self.namesp = namespace
		self.joined = []

	def findCaps(self):
		'''
		gathers nodes of names and get their parameters.
		'''
				
		pubk = findNameUID('_pub_msg_UID_')
		subk = findNameUID('_sub_msg_UID_')

		if subk != None:
			for xc in subk:
				msgs = rospy.get_param(xc)
				isfa = xc.find('_sub')
				isaf = xc.find('UID_')
				namde = xc[:isfa]
				UID = xc[isaf+4:]
				lots = subList(namde,UID,msgs)
				self.sublist.append(lots)

		if pubk != None:
			for xc in pubk:
				msgs = rospy.get_param(xc)
				isfa = xc.find('_pub')
				isaf = xc.find('UID_')
				namde = xc[:isfa]
				UID = xc[isaf+4:]
				lots = pubList(namde,UID,msgs)
				self.publist.append(lots)


	
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
			

	def msgToName_pub(self, namen):
		'''
		returns the msg from a publisher
		'''
		for i in self.publist:
			if i.fut == namen:
				return i.msg
		return None;
	
	def msgToName_sub(self, namen):
		'''
		returns the msg from a subscriber
		'''
		for i in self.sublist:
			if i.fut == namen:
				return i.msg
		return None;

	def subToMsg(self, msgn):
		cats = []
		for i in self.sublist:
			if i.msg == msgn :
				cats.append(i.fut)
		if cats == []:
			return None
		else:
			return cats

	def pubToMsg(self, msgn):
		cats = []
		for i in self.publist:
			if i.msg == msgn:
				cats.append(i.fut)
		if cats == []:
			return None
		else:
			return cats

	def pubToName(self, namen):
		for i in self.publist:
			if i.fut == namen:
				return True
		return False

	def subToName(self, namen):
		for i in self.sublist:
			if i.fut == namen:	
				return True	
		return False

	def both(self):
		start = []
		for x in self.publist:
			start.append(x.name)
		for x in self.sublist:
			start.append(x.name)
		if start == []:
			return None
		else:
			return start


	def pub(self):
		start = []
		for x in self.publist:
			start.append(x.name)
		if start == []:
			return None
		else:
			return start

	def sub(self):
		start = []
		for x in self.sublist:
			start.append(x.name)
		if start == []:
			return None
		else:
			return start
		

	def joinable_pub(self, namen):
		msd = self.msgToName_pub(namen)
		if (msd!=None):
			return self.subToMsg(msd)
		else:
			return None

	def joinable_sub(self, namen):
		msd = self.msgToName_sub(namen)
		if (msd!=None):
			return self.pubToMsg(msd)
		else:
			return None


	def join(self, subnList, pubnList, topic=None):
		if topic == None:
			topic = 'not%i' %(random.randint(1,999))
		subname = subnList
		pubname = pubnList
		rospy.set_param(pubname, topic)
		rospy.set_param(subname, topic)
		if self.hasTopic(topic):
			self.addPubToXTopic(topic,pubname)
			self.addSubToXTopic(topic,subname)
		else:
			art = joinedob()
			art.topic = topic
			art.addPub(pubname)
			art.addSub(subname)
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
		if rospy.has_param(name):
			rospy.delete_param(name)
		self.removeNameJ(name)

	def talk_nodes(self, sub, pub):
		rand = 'not%i' %(random.randint(1,99999))
		rospy.set_param(pub, rand)
		rospy.set_param(sub, rand)

	
	def disconnectAll(self):
		try:
			ns = make_global_ns('ont')
			names = rosgraph.Master(make_caller_id('rosparam-%s'%os.getpid())).getParamNames()
			names.sort()
			plato = [n for n in names if n.startswith(ns)]

			for s in plato:
				if s.find('_msg_'):
					pass
				else:
					rospy.delete_param(s)
			self.joined = []
		except socket.error:
			raise RosParamIOException("Unable to communicate with master!")	
					
		
		


