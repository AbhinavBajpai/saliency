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
  features_pub_ = _n->advertise<sscrovers_pmslam_common::extraFeatures>("/saliency/features_added", 1);

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
		sscrovers_pmslam_common::extraFeatures out;
		out.number = readIn.number;
		std::vector<cv::Point> vertexs;
		cv_bridge::CvImagePtr cv_ptr;
		cv::Mat image_, image2_;
		test=false;
		int size = readIn.number;

		int i=0;
		for(int i=0;i<size;i++){
			vertexs.clear();
			sscrovers_pmslam_common::extraFeature tempExtra;
			try{
				cv_ptr = cv_bridge::toCvCopy(readIn.imgs[i], sensor_msgs::image_encodings::MONO8);
			}
			catch (cv_bridge::Exception& e){
				ROS_ERROR("cv_bridge exception: %s", e.what());
			}
			cv_ptr->image.copyTo(image_);
			cv::Size orig = image_.size();
			cv::resize(image_,image_,cv::Size(orig.width*10,orig.height*10), cv::INTER_NEAREST);
			//ROS_INFO("thinning ");
			thinning(image_);
			
			cv::imshow("sa",image_);
			cv::waitKey(10);
			
			const int BLOCK_SIZE = 3;
			int rowBlocks = image_.rows;
			int colBlocks = image_.cols;    
			int now = 0;
			for(int i = 1; i < rowBlocks-1; i++){
			    for(int j = 1; j < colBlocks-1; j++){
			    //look thro image
				if(image_.at<int>(j,i) == 1){
				//if centre point
					for(int k=-1;k<2;k++){
						for(int l=-1;l<2;l++){
						//search local area
							if(image_.at<int>(j+k,i+l)==1){
								now++;
							}
						}
					}
					if(now > 3 || now<3){
						vertexs.push_back(cv::Point((j/10.0),(i/10.0)));	
					}
				}	
			    }
			}

			//close, not too
			for(int g=0;g<vertexs.size();g++){
				for(int h=g+1;h<vertexs.size();h++){
					float dist = (vertexs.at(g).x - vertexs.at(h).x)*(vertexs.at(g).x - vertexs.at(h).x) + (vertexs.at(g).y - vertexs.at(h).y)*(vertexs.at(g).y - vertexs.at(h).y);
					if (dist < 15){
						vertexs.erase(vertexs.begin()+g);
					}
				}
			}

			for(int p=0;p<vertexs.size();p++){
				vertexs.at(p).x = vertexs.at(p).x + before.poses[i].position.x;
				vertexs.at(p).y = vertexs.at(p).y + before.poses[i].position.y;
			}

			//convert to extraFeature
			tempExtra.point1.x = before.poses[i].position.x;
			tempExtra.point1.y = before.poses[i].position.y;
			tempExtra.numbers=vertexs.size();
			for(int o=0; o<vertexs.size(); o++){
				geometry_msgs::Point32 ptemp;
				ptemp.x = vertexs.at(o).x;
				ptemp.y = vertexs.at(o).y;
				tempExtra.extraPoints.push_back(ptemp);
			}
			out.extras.push_back(tempExtra);
		}
		features_pub_.publish(out);
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
