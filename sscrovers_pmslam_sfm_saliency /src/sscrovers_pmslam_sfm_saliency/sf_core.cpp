#include "sf_core.h"
#include <iostream>
#include <sstream>
#include <fstream>

using namespace std;

SFCore::SFCore(ros::NodeHandle *_n)
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
  ptpairs_sub_ = _n->subscribe("unadjusted", 1, &SFCore::ptpairsCallBack, this);
  featureMap_sub_ = _n->subscribe("/saliency/features_added", 1, &SFCore::extraFeatureCallback, this);
  

  //pmslam functional module

  step_ = -1;
  test=false;
}

SFCore::~SFCore()
{
}


void SFCore::extraFeatureCallback(const sscrovers_pmslam_common::extraFeatures& msg){
	readIn = msg;
	process();
}


void SFCore::process()
{
	if(test){
		test=false;
		if(prev.extras.size()>0){
			prev = readIn;
		}else{
			now = readIn;
			for(int i=0;i<ptpairs_msg_.pairs.size();i++){
				if(ptpairs_msg_.pairs[i].frame==-1){
					rearranged_n.extras.push_back(now.extras[i]);
					rearranged_n1.extras.push_back(prev.extras[ptpairs_msg_.pairs[i].id]);
				}
			}
			fundamentalMatrix(rearranged_n,rearranged_n1);
			prev = now;
		}

		
	}
}

void SFCore::fundamentalMatrix(sscrovers_pmslam_common::extraFeatures& n, sscrovers_pmslam_common::extraFeatures& n1){
	std::vector<cv::Point2f> points1, points2;
	for(int i=0;i<n.extras.size();i++){
		points1.push_back(cv::Point2f(n.extras[i].extraPoints[0].x,n.extras[i].extraPoints[0].y));
		points2.push_back(cv::Point2f(n1.extras[i].extraPoints[0].x,n1.extras[i].extraPoints[0].y));

		points1.push_back(cv::Point2f(n.extras[i].extraPoints[1].x,n.extras[i].extraPoints[1].y));
		points2.push_back(cv::Point2f(n1.extras[i].extraPoints[1].x,n1.extras[i].extraPoints[1].y));

		points1.push_back(cv::Point2f(n.extras[i].extraPoints[2].x,n.extras[i].extraPoints[2].y));
		points2.push_back(cv::Point2f(n1.extras[i].extraPoints[2].x,n1.extras[i].extraPoints[2].y));

		points1.push_back(cv::Point2f(n.extras[i].extraPoints[3].x,n.extras[i].extraPoints[3].y));
		points2.push_back(cv::Point2f(n1.extras[i].extraPoints[3].x,n1.extras[i].extraPoints[3].y));

		points1.push_back(cv::Point2f(n.extras[i].extraPoints[4].x,n.extras[i].extraPoints[4].y));
		points2.push_back(cv::Point2f(n1.extras[i].extraPoints[4].x,n1.extras[i].extraPoints[4].y));

		points1.push_back(cv::Point2f(n.extras[i].extraPoints[5].x,n.extras[i].extraPoints[5].y));
		points2.push_back(cv::Point2f(n1.extras[i].extraPoints[5].x,n1.extras[i].extraPoints[5].y));

	}

	cv::Mat fundamental_matrix = cv::findFundamentalMat(points1, points2, cv::FM_RANSAC, 3, 0.9);
}



void SFCore::ptpairsCallBack(const sscrovers_pmslam_common::unadjustedPairsConstPtr& msg)
{
  step_ = msg->header.stamp.nsec;
  ptpairs_msg_ = *msg;
  test=true;
}



//-----------------------------MAIN------------------------------
int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "SFM");
  ros::NodeHandle n;

  // Create a new NodeExample object.
  SFCore *sf_core = new SFCore(&n);

  //until we use only img callback, below is needless
  // Tell ROS how fast to run this node.
  ros::Rate r(sf_core->rate_);

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
