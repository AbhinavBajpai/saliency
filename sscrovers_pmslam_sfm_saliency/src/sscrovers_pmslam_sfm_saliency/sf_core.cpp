#include "sf_core.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <boost/lexical_cast.hpp>

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
  first = true;

  //pmslam functional module

  step_ = -1;
  test=false;
}

SFCore::~SFCore()
{
}


void SFCore::extraFeatureCallback(const sscrovers_pmslam_common::extraFeatures& msg){
//ROS_INFO("extra feature found");
	readIn = msg;
}


void SFCore::process()
{
	if(test){
	//ROS_INFO("process");
		test=false;
		if(first){
			prev = readIn;
			first = false;
		}else{
			now = readIn;
			//ROS_INFO("shift arrangement, %lu",ptpairs_msg_.pairs.size());
			for(int i=0;i<ptpairs_msg_.pairs.size();i++){
				if(ptpairs_msg_.pairs[i].frame==-1){
					rearranged_n.extras.push_back(now.extras[i]);
					rearranged_n1.extras.push_back(prev.extras[ptpairs_msg_.pairs[i].id]);
				}
			}
			//ROS_INFO("start fundamental matrix");
			fundamentalMatrix(rearranged_n,rearranged_n1);
			rearranged_n.extras.clear();
			rearranged_n1.extras.clear();
			prev = now;
		}

		
	}
}

void SFCore::fundamentalMatrix(sscrovers_pmslam_common::extraFeatures& n, sscrovers_pmslam_common::extraFeatures& n1){
	std::vector<cv::Point2f> points1, points2;
	points1.clear();
	points2.clear();
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
	
	//test
	cv::Mat cmN = cv::Mat::zeros(240,640, CV_8UC1);
	for(int w=0;w<points1.size();w++){

		std::string s = boost::lexical_cast<string>(w);
		cv::Point p1(points1[w].x,points1[w].y);
		//cv::putText(cmN,s,p1, cv::FONT_HERSHEY_PLAIN,0.5,cv::Scalar(255));
		cv::Point p2(points2[w].x+320,points2[w].y);
		//cv::putText(cmN,s,p2, cv::FONT_HERSHEY_PLAIN,0.5,cv::Scalar(255));
		cv::line(cmN, p1, p2, cv::Scalar(255));
	}

	cv::imshow("asd", cmN);
	
	cv::waitKey(3);


	cv::Mat fundamental_matrix = cv::findFundamentalMat(points1, points2, cv::FM_LMEDS, 1, 0.99);

	cv::Mat K =  cv::Mat::ones(3, 3, CV_64F);
	K.at<double>(0,0) = 757.023586;
	K.at<double>(0,1) = 0;
 	K.at<double>(0,2) = 294.561534;
	K.at<double>(1,0) = 0;
	K.at<double>(1,1) = 755.570942;
	K.at<double>(1,2) = 206.390462;
	K.at<double>(2,0) = 0;
	K.at<double>(2,1) = 0;
	K.at<double>(2,2) = 1;
	
	cv::Mat E = /*K.t() */ fundamental_matrix /* K*/;

	cv::SVD svd(E);
	cv::Matx33d W(0,-1,0,   
	1,0,0,
	0,0,1);
	cv::Matx33d Winv(0,1,0,
	-1,0,0,
	0,0,1);
	cv::Mat R = svd.u * cv::Mat(W) * svd.vt; 
	cv::Mat t = svd.u.col(2); 


	ROS_INFO("t = %f %f %f", t.at<double>(0,0), t.at<double>(0,1), t.at<double>(0,2));
}

void SFCore::ptpairsCallBack(const sscrovers_pmslam_common::unadjustedPairsConstPtr& msg)
{
//ROS_INFO("ptpairs found");
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
    sf_core->process();
    ros::spinOnce();
    r.sleep();

  }

  //ros::spin();

  return 0;
} // end main()
