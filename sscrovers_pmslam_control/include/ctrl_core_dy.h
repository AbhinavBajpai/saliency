#ifndef FD_CORE_H
#define FD_CORE_H

//OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv.h>

// ROS includes
#include "ros/ros.h"
#include "ros/time.h"
#include "tf/tf.h"
#include "angles/angles.h"
#include "geometry_msgs/Quaternion.h"

// ROS img handling
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// ROS messages
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include "dynamic/rosdy.h"

namespace enc = sensor_msgs::image_encodings;

// Dynamic reconfigure includes.
//#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
//#include <node_example/node_example_paramsConfig.h>

static const char WINDOW[] = "Image window";

class CtrlCore
{
public:
  //! Constructor.
  CtrlCore(ros::NodeHandle *_n);

  //! Destructor.
  ~CtrlCore();

  //! rate for main loop
  int rate_;

  //! Everything in this function is processed in main loop
  void process();

  //! wait until receive data from all sources
  bool waitForData();

  //! reset time before use waitForData()
  void resetTime();

  void odomSubscribe(); 

  dySubscriber<nav_msgs::Odometry> odom_ds;
  dySubscriber<sensor_msgs::Image> image_ds;

  dyPublisher<sensor_msgs::Image> image_dp;
  dyPublisher<nav_msgs::Path> real_traj_dp;
  dyPublisher<nav_msgs::Path> est_traj_dp;
  dyPublisher<nav_msgs::Path> in_traj_dp;
  dyPublisher<geometry_msgs::PoseStamped> ctrl_vec_dp;

  //! Variable to identify invocation of ROS topic call
  bool odom_callback_inv, img_callback_inv;

  //! current position of the robot
  geometry_msgs::PoseStamped ctrl_vec_msg_, curr_pose_msg_;

  //! images
  cv_bridge::CvImagePtr cv_input_img_ptr_, cv_output_img_ptr_;

  //! other flags
  bool disp_input_image_;

private:
  //! names for topics
  std::string pub_image_topic_name_, pub_real_traj_topic_name_, pub_est_traj_topic_name_, pub_in_traj_topic_name_,
              pub_ctrl_vec_topic_name_, sub_image_topic_name_, sub_odom_topic_name_;

  //! time measurement
  ros::Time time_;
  ros::Duration wait_time_;



  //! sequence step
  long long int curr_step_; //TODO remove dependencies on step!

  //! node handler for image operations
  image_transport::ImageTransport it_;



  //! input image
  //IplImage *output_image_;



  //! true trajectory
  nav_msgs::Path true_trajectory_msg_;

  //! estimated trajectory
  nav_msgs::Path est_trajectory_msg_;

  //! input trajectory
  nav_msgs::Path in_trajectory_msg_;

  //! position noises
  double xy_noise_;
  double z_noise_;
  double yaw_noise_;

  //! camera pitch angle
  //double camera_pitch_;
  //! camera z position
  //double camera_z_;

  //! Function for current odometry information processing
  void processOdometry();

  //! Function for trajectories updating
  void processTrajectory();

  //! Function for image message processing
  void processImage();

  //! Function for publishing image data
  void publishImage();

  //! Function for publishing travelled trajectory data
  void publishTrajectories();

  //! Function for publishing current control vector
  void publishCtrlVector();

  //! callback function for image subscription
  void imageCallBack(const sensor_msgs::ImageConstPtr& msg);

  //! callback function for odometry subscription
  void odomCallBack(const nav_msgs::OdometryConstPtr& msg);


};

#endif //FD_CORE_H
