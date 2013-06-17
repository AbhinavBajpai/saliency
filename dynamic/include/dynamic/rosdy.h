#ifndef ROSDY_H
#define ROSDY_H

#include "ros/ros.h"

#include <string>
#include <iostream>
#include <cxxabi.h>
#include <boost/bind.hpp>
#include <boost/utility.hpp>
#include <sstream>
using namespace std;

void pubCap(string name, string UID, string data_class) {
	ros::param::set(name + "_pub_msg_UID_" + UID, data_class);
}
void subCap(string name, string UID, string data_class) {
	ros::param::set(name + "_sub_msg_UID_" + UID, data_class);
}



template <class M>
class dyPublisher 
{
  public:
	dyPublisher(){}
	dyPublisher(string UIDin, ros::NodeHandle n){
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
	string topic;
	string data_class;
	string name;
	string UID;
	void onShutdown(){
		if (nh.hasParam(name + "_UID_" + UID)){
			nh.deleteParam(name + "_UID_" + UID);
		}
		if (nh.hasParam(name + "_pub_msg_UID_" + UID)){
			nh.deleteParam(name + "_pub_msg_UID_" + UID);
		}
	}
	bool checkForTopic(){
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
	void publish(const M& message){
		pubg.publish(message);
	}

	void publishOnce(const M& message){
		if(checkForTopic()){
			pubg.publish(message);
			ros::spinOnce();
		}
	}

	void publishOnceWait(const M& message){
		bool ret = true;
		while(ros::ok() && ret){
			if(checkForTopic()){
				pubg.publish(message);
				ret = false;
				ros::spinOnce();
			}
		}
	}

	void changeTopic(){
		if (set == true){
			//delete pubg;
			pubg = nh.advertise<M>(topic, 10);
		}else{
			pubg = nh.advertise<M>(topic, 10);
		}
	}
	void pubap(){
		nh.setParam("as",5);
	}


  private:
	bool set;
	ros::Publisher pubg;
	ros::NodeHandle  nh;
};


template <class M>
class dySubscriber 
{
  public:
	dySubscriber(){}

	dySubscriber(string UIDin, ros::NodeHandle n){
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
		first = true;
		second = true;
		change = false;
		stop = false;
	}

	string topic;
	string data_class;
	string name;
	string UID;
	bool set;
	ros::Subscriber subg;
	ros::NodeHandle  nh;
	bool first;
	bool second;
	bool test;
	bool change;
	bool stop;



	void onShutdown(){
		if (nh.hasParam(name + "_UID_" + UID)){
			nh.deleteParam(name + "_UID_" + UID);
		}
		if (nh.hasParam(name + "_sub_msg_UID_" + UID)){
			nh.deleteParam(name + "_sub_msg_UID_" + UID);
		}
	}
	bool checkForTopic(){
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
	
	

	void subscribe(void(*fp)(const boost::shared_ptr<M const>&)){
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
						ros::spinOnce();
					}
				}
			}
		}
	}


	void subscribe(const boost::function<void (const boost::shared_ptr<M const>&)>& callback)
{
	while (ros::ok()){
		test = checkForTopic();
		first = true;
		second = true;
		if(test){
			while(ros::ok() and second){
				subg = nh.subscribe(topic, 1000, callback);
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
					ros::spinOnce();
				}
			}
		}
	}
}

	template<class T>
	void subscribe(void(T::*fp)(M), T* obj)
{
	while (ros::ok()){
		test = checkForTopic();
		first = true;
		second = true;
		if(test){
			while(ros::ok() and second){
				subg = nh.subscribe(topic, 1000, &fp, obj);
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
					ros::spinOnce();
				}
			}
		}
	}
}

	template<class T>
	void subscribe(void(T::*fp)(M) const, T* obj)
{
	while (ros::ok()){
		test = checkForTopic();
		first = true;
		second = true;
		if(test){
			while(ros::ok() and second){
				subg = nh.subscribe(topic, 1000, &fp, obj);
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
					ros::spinOnce();
				}
			}
		}
	}
}

	template<class T>
	void subscribe(void(T::*fp)(const boost::shared_ptr<M const>&) const, T* obj)
{
	while (ros::ok()){
		test = checkForTopic();
		first = true;
		second = true;
		if(test){
			while(ros::ok() and second){
				subg = nh.subscribe(topic, 1000, &fp, obj);
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
					ros::spinOnce();
				}
			}
		}
	}
}


void subscribeOnce(void(*fp)(const boost::shared_ptr<M const>&)){
	test = checkForTopic();
	if(test){
		subg = nh.subscribe(topic, 1000, fp);
		ros::spinOnce();
	}
}
void subscribeOnceHold(void(*fp)(const boost::shared_ptr<M const>&)){
	test = checkForTopic();
	if(test){
		subg = nh.subscribe(topic, 1000, fp);
	}
}

void subscribeOnce(const boost::function<void (const boost::shared_ptr<M const>&)>& callback)
{
	test = checkForTopic();
	if(test){
		subg = nh.subscribe(topic, 1000, callback);
		ros::spinOnce();
	}
}

void subscribeTillOnce(const boost::function<void (const boost::shared_ptr<M const>&)>& callback)
{
	bool sd = true;
	while(ros::ok() && sd){
		test = checkForTopic();
		if(test){
			subg = nh.subscribe(topic, 1000, callback);
			ros::spinOnce();
			sd = false;
		}
	}
}

void subscribeOnceHold(const boost::function<void (const boost::shared_ptr<M const>&)>& callback)
{
	test = checkForTopic();
	if(test){
		subg = nh.subscribe(topic, 1000, callback);
	}
}

};


#endif
