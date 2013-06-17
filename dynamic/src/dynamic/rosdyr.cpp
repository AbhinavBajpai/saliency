#include "ros/ros.h"
#include "std_msgs/Int8.h"

#include <string>
#include <iostream>
#include <cxxabi.h>

#include "rosdy.h"

#include <mutex>

#include <sstream>
using namespace std;

void pubCap(string name, string UID, string data_class) {
	ros::param::set(name + "_pub_msg_UID_" + UID, data_class);
}
void subCap(string name, string UID, string data_class) {
	ros::param::set(name + "_sub_msg_UID_" + UID, data_class);
}

template <class M>
dyPublisher<M>::dyPublisher(string UIDin, ros::NodeHandle n){
	int status;
	char * demangled = abi::__cxa_demangle(typeid(M).name(),0,0,&status);
	stringstream ss;
	ss << demangled;
	ss >> data_class;
	int fy1 = data_class.find("::");
	int fy2 = data_class.find("_<");
	data_class = data_class.substr(fy1+2,fy2-fy1-2);
	string s = ros::this_node::getName();
	name = s.substr(1);
	UID = UIDin;
	nh = n;
	pubCap(name,UID,data_class);
}

template <class M>
void dyPublisher<M>::onShutdown(){
	if (nh.hasParam(name + "_UID_" + UID)){
		nh.deleteParam(name + "_UID_" + UID);
	}
	if (nh.hasParam(name + "_pub_msg_UID_" + UID)){
		nh.deleteParam(name + "_pub_msg_UID_" + UID);
	}
}

template <class M>
bool dyPublisher<M>::checkForTopic(){
	string topic_test = "";
	if (nh.hasParam(name + "_UID_" + UID)){
		nh.getParam(name + "_UID_" + UID, topic_test);
		if (topic != topic_test) {
			topic = topic_test;
			changeTopic();
			set = true;
		}
		return set;
	}else{
		topic = "";
		if (set == true){
			set = false;
			//delete pubg;
		}else{
			set = false;
		}

		return set;
	}
}

template <class M>
void dyPublisher<M>::publish(const M& message){
	pubg.publish(message);
}

template <class M>
void dyPublisher<M>::changeTopic(){
	if (set == true){
		//delete pubg;
		pubg = nh.advertise<M>(topic, 10);
	}else{
		pubg = nh.advertise<M>(topic, 10);
	}
}
template <class M>
void dyPublisher<M>::pubap(){
	nh.setParam("as",5);
}




template <class M>
dySubscriber<M>::dySubscriber(string UIDin, ros::NodeHandle n, mutex * mutag){
	int status;
	char * demangled = abi::__cxa_demangle(typeid(M).name(),0,0,&status);
	stringstream ss;
	ss << demangled;
	ss >> data_class;
	int fy1 = data_class.find("::");
	int fy2 = data_class.find("_<");
	data_class = data_class.substr(fy1+2,fy2-fy1-2);
	string s = ros::this_node::getName();
	name = s.substr(1);
	UID = UIDin;
	nh = n;
	pubCap(name,UID,data_class);
	lockable = true;
	muta = mutag;
	first = true;
	second = true;
	change = false;
	stop = false;
}
template <class M>
dySubscriber<M>::dySubscriber(string UIDin, ros::NodeHandle n){
	int status;
	char * demangled = abi::__cxa_demangle(typeid(M).name(),0,0,&status);
	stringstream ss;
	ss << demangled;
	ss >> data_class;
	int fy1 = data_class.find("::");
	int fy2 = data_class.find("_<");
	data_class = data_class.substr(fy1+2,fy2-fy1-2);
	string s = ros::this_node::getName();
	name = s.substr(1);
	UID = UIDin;
	nh = n;
	subCap(name,UID,data_class);
	lockable=false;
	first = true;
	second = true;
	change = false;
	stop = false;
}

template <class M>	
void dySubscriber<M>::onShutdown(){
	if (nh.hasParam(name + "_UID_" + UID)){
		nh.deleteParam(name + "_UID_" + UID);
	}
	if (nh.hasParam(name + "_pub_msg_UID_" + UID)){
		nh.deleteParam(name + "_sub_msg_UID_" + UID);
	}
}

template <class M>
bool dySubscriber<M>::checkForTopic(){
	if(lockable){
		muta.lock();
		string topic_test = "";
		if (nh.hasParam(name + "_UID_" + UID)){
			nh.getParam(name + "_UID_" + UID, topic_test);
			if (topic != topic_test) {
				topic = topic_test;
				set = true;
				change = true;
			}else{
				set = true;
				change = false;
			}
			return set;
		}else{
			topic = "";
			if (set == true){
				set = false;
				stop= true;
				//delete pubg;
			}else{
				set = false;
				stop = false;
			}

			return set;
		}
		muta.unlock();
	}else{
		string topic_test = "";
		if (nh.hasParam(name + "_UID_" + UID)){
			nh.getParam(name + "_UID_" + UID, topic_test);
			if (topic != topic_test) {
				topic = topic_test;
				set = true;
				change = true;
			}else{
				set = true;
				change = false;
			}
			return set;
		}else{
			topic = "";
			if (set == true){
				set = false;
				stop= true;
				//delete pubg;
			}else{
				set = false;
				stop = false;
			}

			return set;
		}
	}
}
	
	
template <class M>
void dySubscriber<M>::subscribe(void(*fp)(M)){	
	while (ros::ok()){
		test = checkForTopic();
		first = true;
		second = true;
		if(test){
			while(ros::ok() and second){
				subg = nh.subscribe(topic, 1000, fp);
				first=true;
				while(ros::ok() and first and second){
					test = checkForTopic();
					if (change) {
						//subg.unregister();
						//subg.~subscribe();
						first = false;
					}
					if (stop) {
						//subg.unregister();
						//subg.~subscribe();
						stop =false;
						first = false;
						second =false;
					}
					ros::WallDuration timeout(0.1f);
				}
			}
		}
	}
}




int main(int argc, char **argv){
	ros::init(argc, argv, "test");
	ros::NodeHandle n;
	dyPublisher <std_msgs::Int8> dp("testPub", n);


	int count = 0;
	while (ros::ok()){
		if (dp.checkForTopic()) {
			cout << "asd"<< endl;
		}else{
			cout << "sadd"<< endl;
		}
	}
}




