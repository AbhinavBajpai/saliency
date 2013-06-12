#ifndef FD_CORE_H
#define FD_CORE_H

// ROS includes
#include "ros/ros.h"
#include "ros/time.h"

//OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h> //if needed just only for electric
#if (CV_MAJOR_VERSION >= 2) && (CV_MINOR_VERSION >= 4)//! needed for backward compatibility from 2.4
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#endif

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

//ros messages
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include "sscrovers_pmslam_common/DynamicArray.h"

#include "dynamic/rosdy.h"

#include "surf_fd.h"//move the extraction there?
namespace enc = sensor_msgs::image_encodings;

class FDCore
{
public:
  //! Constructor.
  FDCore(ros::NodeHandle *_n);

  //! Destructor.
  ~FDCore();

  //! rate for main loop
  int rate_;

  //! process everything in main loop
  void process();

  //! images
  cv_bridge::CvImagePtr cv_input_img_ptr_, cv_output_img_ptr_;

  dySubscriber<sensor_msgs::Image> image_ds;
  dyPublisher<sscrovers_pmslam_common::DynamicArray> features_dp;

  void publishImageDy();

private:

  //!flags
  //! debug only - save features before send it to topic
  bool features_to_file_f_;
  //! display image with founded features
  bool disp_img_f_;
  //! publish image with marked features
  bool pub_output_image_f_;

  //! name for topics
  std::string in_img_topic_name_, out_img_topic_name_, out_features_topic_name_;

  //! publishers
  ros::Publisher features_pub_;

  //! dynamic serialized array for publishing features
  sscrovers_pmslam_common::DynamicArray features_msg_;

  //! node handler for image operations
  image_transport::ImageTransport it_;

  //! image publisher
  image_transport::Publisher image_pub_;

  //! image subscriber
  image_transport::Subscriber image_sub_;

  //! contains a sequence of image keypoints for OpenCV SURF.
  CvSeq *keypoints_;

  //! contains a sequence of keypoint descriptors for OpenCV SURF.
  CvSeq *descriptors_;

  //! Object of functional class
  //SurfFD surf_fd_;

  /*!
   * \brief OpenCV SURF
   *
   * \param m image
   */
  void extractKeypoints(cv::Mat* input_image);

  //! prepare image message and publish it
  void publishImage();

  //! prepare features message and publish it
  void publishFeatures();
  void publishFeaturesDy();

  //! callback for image subscribing
  void imageCallBack(const sensor_msgs::ImageConstPtr& msg);

};

#endif //FD_CORE_H
