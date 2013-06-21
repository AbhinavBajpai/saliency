/*
 * pangu_seq_pub_core.h
 *
 * Created on: 19 Jul 2012
 * Author: Piotr Weclewski (weclewski.piotr@gmail.com)
 *
 * Declaration of node responsible for read offline data and publish image and odometry as a real rover
 */

#ifndef FILE_SEQ_PUB_CORE_H_
#define FILE_SEQ_PUB_CORE_H_

//STL
#include <string>
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <vector>

//OpenCV
#include <cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// ROS includes
#include "ros/ros.h"
#include "ros/time.h"
#include "ros/package.h"
#include "tf/tf.h"
//#include "angles/angles.h"
#include "geometry_msgs/Quaternion.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include "dynamic/rosdy.h"

namespace enc = sensor_msgs::image_encodings;

using namespace std;
using namespace cv;

typedef vector<vector<string> > Rows_t;

class FileSeqPubCore
{
public:
  FileSeqPubCore(ros::NodeHandle *_n);
  virtual ~FileSeqPubCore();

  //! rate for main loop [Hz]
  double rate_;

  //! node processing function
  void process();

  //! step for acquire images from files
  int step_;

  //! play back everything continuously in the loop
  bool loop_f_;

  //! for play back only one frame in the loop
  int one_frame_;

  //! names for topics
  std::string pub_image_topic_name_, pub_odom_topic_name_;

  int start_no_;

  //! absolute paths for images and trajectory file (default in: {sequence_publisher_path}/input)
  std::string path_to_images_, path_to_poses_;

  std::string trajectory_file_name_;

  //! image file prefix (eg. if image file are named "frame_orig_1000000.jpg" this should be "frame_orig_")
  std::string image_file_name_prefix_;

  //! extension for image file - is allowed every image type that opencv can handle
  std::string image_extension_;

  //! node handler for image operations
  image_transport::ImageTransport it_;

  //! image publisher
  image_transport::Publisher image_pub_;

  //! odometry publisher
  ros::Publisher odom_pub_;

  //! image
  cv_bridge::CvImagePtr cv_img_ptr_;

  //! odometry
  nav_msgs::Odometry odom_msg_;

  //! temporary structure for reading trajectory from file
  Rows_t temp_trajectory_;

  //! make image file name from prefix, current index and extension
  std::string makeFileName(const std::string& basename, const int& index, const std::string& ext);

  //! load trajectory from file
  void loadTrajectory();

  //! get position from loaded trajectory
  void acquirePose();

  //! get Image from file
  void acquireImage();

  //! publish image
  void publishImage();

  //! publish odometry information
  void publishOdom();

  //dp
  dyPublisher<sensor_msgs::Image> image_dp;
  dyPublisher<nav_msgs::Odometry> odom_dp;
};

#endif /* PANGU_SEQ_PUB_CORE_H_ */
