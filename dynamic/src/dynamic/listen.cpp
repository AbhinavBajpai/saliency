#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include <string>
#include <iostream>

#include "dynamic/rosdyp.h"


void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv){
	ros::init(argc, argv, "test");
	ros::NodeHandle n;
	string hh = "testSub";
	dySubscriber<std_msgs::String> ds(hh, n);
	ds.subscribe(chatterCallback);
	ds.onShutdown();
}
