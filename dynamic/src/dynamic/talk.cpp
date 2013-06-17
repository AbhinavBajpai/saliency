#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include <string>
#include <iostream>

#include "dynamic/rosdyp.h"


int main(int argc, char **argv){
	ros::init(argc, argv, "tsdt");
	ros::NodeHandle n;
	std::string hh = "testPub";
	dyPublisher<std_msgs::String> dp(hh, n);
	ros::Rate loop_rate(10);

	int count = 0;
	while (ros::ok()){
		if (dp.checkForTopic()) {
			std_msgs::String msg;

			std::stringstream ss;
			ss << "hello world " << count;
			msg.data = ss.str();

			ROS_INFO("%s", msg.data.c_str());
			dp.publish(msg);
			ros::spinOnce();
			++count;
			
		}else{
			ROS_INFO("not started");
		}
		loop_rate.sleep();
	}
	dp.onShutdown();
}
