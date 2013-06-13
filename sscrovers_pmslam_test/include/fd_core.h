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
#include "sensor_msgs/fill_image.h"
#include <geometry_msgs/PoseStamped.h>
#include "sscrovers_pmslam_common/SALVector.h"
#include "sscrovers_pmslam_common/SPoint.h"
#include "sscrovers_pmslam_common/PtPairs.h"
#include <nav_msgs/Path.h>


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
  std::vector <sscrovers_pmslam_common::SPoint> sal_db;


private:

  //!flags
  //! debug only - save features before send it to topic
  bool features_to_file_f_;
  //! display image with founded features
  bool disp_img_f_;
  //! publish image with marked features
  bool pub_output_image_f_;

  //! image publisher
  image_transport::Publisher image_pub_;

  //! name for topics
  std::string in_img_topic_name_, out_img_topic_name_, out_features_topic_name_;

  //! Subscriber
  ros::Subscriber ptpairs_sub_, features_db_sub_;

//! topic names
  std::string pub_pt_topic_name_, sub_pt_topic_name_, sub_traj_topic_name_, sub_db_topic_name_, sub_features_topic_name_;

  //! node handler for image operations
  image_transport::ImageTransport it_;

  //! image subscriber
  image_transport::Subscriber image_sub_;

  //! images
  cv_bridge::CvImagePtr cv_input_img_ptr_, cv_output_img_ptr_;

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

  //! prepare image message and publish it
  void publishImage();

  //! callback for image subscribing
  void imageCallBack(const sensor_msgs::ImageConstPtr& msg);

  void ptpairsCallBack(const sscrovers_pmslam_common::PtPairsConstPtr& msg);

  //! Callback function for features database subscription.
  void featuresDBCallBack(const sscrovers_pmslam_common::SALVector& msg);

  int hess_no_;

  void editImage();

  //! point pairs message
  sscrovers_pmslam_common::PtPairs ptpairs_msg_;

};

#endif //FD_CORE_H
