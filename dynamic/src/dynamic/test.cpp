#include "ros/ros.h"
#include "std_msgs/Int8.h"

#include <string>
#include <iostream>

#include "dynamic/rosdyp.h"

using namespace std;

int main(int argc, char **argv){
	ros::init(argc, argv, "test");
	ros::NodeHandle n;
	string hh = "testPub";
	dyPublisher<std_msgs::Int8> dp(hh, n);


	int count = 0;
	while (ros::ok()){
		if (dp.checkForTopic()) {
			cout << "asd"<< endl;
		}else{
			cout << "sadd"<< endl;
		}
	}

}

