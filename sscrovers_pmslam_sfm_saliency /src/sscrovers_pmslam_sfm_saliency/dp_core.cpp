#include "dp_core.h"
#include <iostream>
#include <sstream>
#include <fstream>

using namespace std;

DPCore::DPCore(ros::NodeHandle *_n)
{
  // Initialise node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can be run simultaneously
  // while using different parameters.
  ros::NodeHandle private_node_handle("~");
  private_node_handle.param("rate", rate_, int(10));
  //! topics name
private_node_handle.param("output_features_topic_name", sub_features_topic_name_, string("/saliency/features_Hou"));


  //! publishers
  //features_pub_ = _n->advertise<sscrovers_pmslam_common::extraFeatures>("/saliency/features_added", 1);

  //! subscribers
  featureMap_sub_ = _n->subscribe("/saliency/features_added", 1, &DPCore::extraFeatureCallback, this);
  ptpairs_sub_ = _n->subscribe("ptpairs", 1, &DPCore::ptpairsCallBack, this);

  //pmslam functional module

  step_ = -1;
  test=false;
}

DPCore::~DPCore()
{
}


void DPCore::extraFeatureCallback(const sscrovers_pmslam_common::extraFeatures& msg){
	readIn = msg;
	process();
}


void DPCore::process()
{
	int num=0;
	for(int i=0;i<readIn.extras.size();i++){
		num += readIn.extras[i].extraPoints.size();
	}
	ROS_INFO("%i",num );
}


void DPCore::ptpairsCallBack(const sscrovers_pmslam_common::PtPairsConstPtr& msg)
{
  step_ = msg->header.stamp.nsec;
  ptpairs_msg_ = *msg;
  dbTest=true;
}



//-----------------------------MAIN------------------------------
int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "depth_perception");
  ros::NodeHandle n;

  // Create a new NodeExample object.
  DPCore *dp_core = new DPCore(&n);

  //until we use only img callback, below is needless
  // Tell ROS how fast to run this node.
  ros::Rate r(dp_core->rate_);

  // Main loop.

  while (n.ok())
  {
    //dp_core->Publish();
    ros::spinOnce();
    r.sleep();

  }

  //ros::spin();

  return 0;
} // end main()
