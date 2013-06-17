#include "ros/ros.h"
#include "std_msgs/Int8.h"

#include <string>
#include <iostream>
#include <cxxabi.h>

#include <sstream>
using namespace std;

void pubCap(string name, string UID, string data_class) {
	ros::param::set(name + "_pub_msg_UID_" + UID, data_class);
}


class dyPublisher 
{
  public:

	dyPublisher(string UIDin, ros::NodeHandle n);
	string topic;
	string data_class;
	string name;
	string UID;
	void onShutdown();
	template <class M>
	bool checkForTopic();
	template <typename M>
	void publish(const M& message);
	template <class M>
	void changeTopic();
	void pubap();


  private:
	bool set;
	ros::Publisher pubg;
	ros::NodeHandle  nh;
};

void dyPublisher::pubap(){
	nh.setParam("as",5);
}


dyPublisher::dyPublisher(string UIDin, ros::NodeHandle n){
	string s = ros::this_node::getName();
	name = s.substr(1);
	UID = UIDin;
	nh = n;
	pubCap(name,UID,data_class);
}
 

void dyPublisher::onShutdown(){
	if (nh.hasParam(name + "_UID_" + UID)){
		nh.deleteParam(name + "_UID_" + UID);
	}
	if (nh.hasParam(name + "_pub_msg_UID_" + UID)){
		nh.deleteParam(name + "_pub_msg_UID_" + UID);
	}
}

template <class M>
bool dyPublisher::checkForTopic(){
	int status;
	char * demangled = abi::__cxa_demangle(typeid(M).name(),0,0,&status);
	stringstream ss;
	ss << demangled;
	ss >> data_class;
	string topic_test = "";
	int fy1 = data_class.find("::");
	int fy2 = data_class.find("_<");
	data_class = data_class.substr(fy1+2,fy2-fy1-2);
	if (nh.hasParam(name + "_UID_" + UID)){
		nh.getParam(name + "_UID_" + UID, topic_test);
		if (topic != topic_test) {
			topic = topic_test;
			changeTopic<M>();
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
void dyPublisher::changeTopic(){
	if (set == true){
		//delete pubg;
		pubg = nh.advertise<M>(topic, 10);
	}else{
		pubg = nh.advertise<M>(topic, 10);
	}
}

template <typename M>
void dyPublisher::publish(const M& message){
	pubg.publish(message);
}


int main(int argc, char **argv){
	ros::init(argc, argv, "test");
	ros::NodeHandle n;
	dyPublisher dp("testPub", n);
	//cout << dp.checkForTopic<std_msgs::Int8>() << endl ;
}




