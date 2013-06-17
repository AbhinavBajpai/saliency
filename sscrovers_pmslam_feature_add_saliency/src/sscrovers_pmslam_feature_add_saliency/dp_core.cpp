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
  features_pub_ = _n->advertise<geometry_msgs::PoseArray>("/saliency/features_added", 1);

  //! subscribers
  features_sub_ = _n->subscribe(sub_features_topic_name_.c_str(), 1, &DPCore::featuresCallback, this);
  featureMap_sub_ = _n->subscribe("/saliency/features_Hou_Maps", 1, &DPCore::featureMapCallback, this);

  //pmslam functional module

  step_ = -1;
  test=false;
}

DPCore::~DPCore()
{
}

void DPCore::featuresCallback(const geometry_msgs::PoseArray& msg){

	before = msg;
	test=true;
}

void DPCore::featureMapCallback(const sscrovers_pmslam_common::featureMap& msg){
	readIn = msg;
	process();
}

/**
 * Perform one thinning iteration.
 * Normally you wouldn't call this function directly from your code.
 *
 * @param  im    Binary image with range = 0-1
 * @param  iter  0=even, 1=odd
 */
void thinningIteration(cv::Mat& im, int iter)
{
    cv::Mat marker = cv::Mat::zeros(im.size(), CV_8UC1);

    for (int i = 1; i < im.rows-1; i++)
    {
        for (int j = 1; j < im.cols-1; j++)
        {
            uchar p2 = im.at<uchar>(i-1, j);
            uchar p3 = im.at<uchar>(i-1, j+1);
            uchar p4 = im.at<uchar>(i, j+1);
            uchar p5 = im.at<uchar>(i+1, j+1);
            uchar p6 = im.at<uchar>(i+1, j);
            uchar p7 = im.at<uchar>(i+1, j-1);
            uchar p8 = im.at<uchar>(i, j-1);
            uchar p9 = im.at<uchar>(i-1, j-1);

            int A  = (p2 == 0 && p3 == 1) + (p3 == 0 && p4 == 1) + 
                     (p4 == 0 && p5 == 1) + (p5 == 0 && p6 == 1) + 
                     (p6 == 0 && p7 == 1) + (p7 == 0 && p8 == 1) +
                     (p8 == 0 && p9 == 1) + (p9 == 0 && p2 == 1);
            int B  = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
            int m1 = iter == 0 ? (p2 * p4 * p6) : (p2 * p4 * p8);
            int m2 = iter == 0 ? (p4 * p6 * p8) : (p2 * p6 * p8);

            if (A == 1 && (B >= 2 && B <= 6) && m1 == 0 && m2 == 0)
                marker.at<uchar>(i,j) = 1;
        }
    }

    im &= ~marker;
}

/**
 * Function for thinning the given binary image
 *
 * @param  im  Binary image with range = 0-255
 */
void thinning(cv::Mat& im)
{
    im /= 255;

    cv::Mat prev = cv::Mat::zeros(im.size(), CV_8UC1);
    cv::Mat diff;

    do {
        thinningIteration(im, 0);
        thinningIteration(im, 1);
        cv::absdiff(im, prev, diff);
        im.copyTo(prev);
    } 
    while (cv::countNonZero(diff) > 0);

    im *= 255;
}


void DPCore::process()
{
	if(test){
		cv_bridge::CvImagePtr cv_ptr;
		cv::Mat image_, image2_;
		test=false;
		int size = readIn.number;
		int i=0;
		//for(int i=0;i<size;i++){
			try{
				cv_ptr = cv_bridge::toCvCopy(readIn.imgs[i], sensor_msgs::image_encodings::MONO8);
			}
			catch (cv_bridge::Exception& e){
				ROS_ERROR("cv_bridge exception: %s", e.what());
			}
			cv_ptr->image.copyTo(image_);
			cv::resize(image_,image_,cv::Size(320,320), cv::INTER_LINEAR);
			image2_ = image_.clone();
			ROS_INFO("thinning");
			thinning(image_);
			cv::imshow("Before", image2_);
			cv::imshow("After", image_);
			cv::waitKey(10);
		//}
	}


}


void DPCore::addFeature(cv::Mat *image, float x, float y){
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
