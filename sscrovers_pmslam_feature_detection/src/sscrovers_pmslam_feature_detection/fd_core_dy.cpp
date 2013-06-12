#include "fd_core_dy.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <boost/bind.hpp>


using namespace std;

static const char WINDOW[] = "fd_window";
void imageCallBackDy(const sensor_msgs::Image::ConstPtr& msg, FDCore * fd);


FDCore::FDCore(ros::NodeHandle *_n) :
    it_(*_n)
{

#if (CV_MAJOR_VERSION >= 2) && (CV_MINOR_VERSION >= 4)
  cv::initModule_nonfree();
#endif
  // Initialise node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can be run simultaneously
  // while using different parameters.
  ros::NodeHandle private_node_handle("~");
  private_node_handle.param("rate", rate_, int(10));
  //topics name
  //debug to files storing
  private_node_handle.param("debug_features_to_file", features_to_file_f_, bool(false));
  //display output img
  private_node_handle.param("disp_img", disp_img_f_, bool(true));
  //publish output img
  private_node_handle.param("pub_img", pub_output_image_f_, bool(false));

  //subscribers
  //image_sub_ = it_.subscribe(in_img_topic_name_.c_str(), 1, &FDCore::imageCallBack, this);
  image_ds = dySubscriber<sensor_msgs::Image>("org_image", *_n);

  //publishers
  //features_pub_ = _n->advertise<sscrovers_pmslam_common::DynamicArray>(out_features_topic_name_.c_str(), 1);
  features_dp = dyPublisher<sscrovers_pmslam_common::DynamicArray>("features", *_n);

  //initialize empty images
  cv_input_img_ptr_.reset(new cv_bridge::CvImage);
  cv_output_img_ptr_.reset(new cv_bridge::CvImage);

  //initialize features structures
  keypoints_ = 0, descriptors_ = 0;

  //create window for output image
  if (disp_img_f_)
    cv::namedWindow(WINDOW);

  image_ds.subscribe(boost::bind(imageCallBackDy,_1,this));
  
}

FDCore::~FDCore()
{
  //destroy window for output image
  if (disp_img_f_)
    cv::destroyWindow(WINDOW);
}




void imageCallBackDy(const sensor_msgs::Image::ConstPtr& msg, FDCore * fd){
  try
  {
    fd->cv_input_img_ptr_ = cv_bridge::toCvCopy(msg, enc::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  fd->process();
}


void FDCore::process()
{
  //put here everything that should run with node loop
  extractKeypoints(&cv_input_img_ptr_->image);
  publishImageDy();
  publishFeaturesDy();
}

void FDCore::extractKeypoints(cv::Mat* input_image)
{
  CvSURFParams params = cvSURFParams(500, 0);
  CvMemStorage* storage = cvCreateMemStorage(0);

  IplImage _img(*input_image);
  cvExtractSURF(&_img, 0, &keypoints_, &descriptors_, storage, params);

  //ROS_INFO(keypoints_[0]->hessian);

}

void FDCore::publishImageDy()
{
  cv_output_img_ptr_->header.stamp = cv_input_img_ptr_->header.stamp;

  cv::cvtColor(cv_input_img_ptr_->image, cv_output_img_ptr_->image, CV_GRAY2BGR);

  if (disp_img_f_)
  {
    for (int i = 0; i < keypoints_->total; i++)
    {
      CvSURFPoint* r = (CvSURFPoint*)cvGetSeqElem(keypoints_, i);
      CvPoint center;
      int radius;
      center.x = cvRound(r->pt.x);
      center.y = cvRound(r->pt.y);
      radius = cvRound(r->size * 1.2 / 9. * 2);
      cv::circle(cv_output_img_ptr_->image, center, radius, cvScalar(0, 255, 0), 1, 8, 0);
    }

    cv::imshow(WINDOW, cv_output_img_ptr_->image);
    cv::waitKey(3);
  }


}

void FDCore::publishFeaturesDy()
{
  features_msg_.header.stamp = cv_input_img_ptr_->header.stamp;

  int s = descriptors_->elem_size / sizeof(float);

  features_msg_.dims.clear();
  features_msg_.data.clear();
  features_msg_.types.clear();

  features_msg_.dims.push_back(keypoints_->total);
  features_msg_.dims.push_back(1);
  features_msg_.dims.push_back(s);

  features_msg_.types.push_back("---");
  features_msg_.types.push_back("CvSURFPoint");
  features_msg_.types.push_back("float");

  unsigned int kp_size = features_msg_.dims[0] * features_msg_.dims[1] * sizeof(CvSURFPoint);
  unsigned int ds_size = features_msg_.dims[0] * features_msg_.dims[2] * sizeof(float);

  features_msg_.data.resize(kp_size + ds_size);

  cvCvtSeqToArray(keypoints_, &features_msg_.data[0]);
  cvCvtSeqToArray(descriptors_, (&features_msg_.data[0] + kp_size));

  features_dp.publishOnceWait(features_msg_);

}



//-----------------------------MAIN----------------------------
int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "feature_detection");
  ros::NodeHandle n;
  // Create a new NodeExample object.
  FDCore* __attribute__((unused)) fd_core_ptr_ = new FDCore(&n);

  /*//until we use only img callback, below is needless
   // Tell ROS how fast to run this node.
   ros::Rate r(fd_core->rate);

   // Main loop.

   while (n.ok())
   {
   //fd_core->Process();
   //fd_core->Publish();
   ros::spinOnce();
   r.sleep();
   }
   */
  //ros::spin();


  fd_core_ptr_->image_ds.onShutdown();

  fd_core_ptr_->features_dp.onShutdown();



  return 0;
} // end main()
