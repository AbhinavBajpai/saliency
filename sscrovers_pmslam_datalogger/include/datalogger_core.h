#ifndef DATALOGGER_CORE_H
#define DATALOGGER_CORE_H

// std, stl
#include <cstdlib>
#include <string>
#include <vector>

// ROS includes
#include "ros/ros.h"
#include "ros/time.h"
#include <ros/package.h>
#include "tf/tf.h"

// ROS - opencv handling
#include <cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// messages
#include <nav_msgs/Path.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/image_encodings.h>
#include <sscrovers_pmslam_common/ControlVector.h>
#include <sscrovers_pmslam_common/DynamicArray.h>
#include <sscrovers_pmslam_common/PairedPoints3D.h>
#include <sscrovers_pmslam_common/Map3D.h>
#include <sscrovers_pmslam_common/PMSlamData.h>
#include <sscrovers_pmslam_common/PtPairs.h>
#include <geometry_msgs/PoseStamped.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

// pmslam specific structures - TODO to remove or move to common package
#include "RoverState.h"//to remove?
#include "Map.h"//to remove
#include "SURFPoint.h"

// db template definition - TODO move to common package
#include "features_db.h"

// Dynamic reconfigure includes.
//#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
//#include <node_example/node_example_paramsConfig.h>

//! needed by ROS cv_bridge
namespace enc = sensor_msgs::image_encodings;

using namespace std;

//! core class for pmslam data logger node
class DataloggerCore
{
public:
  //! Constructor.
  DataloggerCore(ros::NodeHandle *_n);

  //! Destructor.
  ~DataloggerCore();

  // frequency of node main loop
  int rate_;
  // current step - to remove or change to time stamp
  int step_;

  //! other combined data reports generation and visualisation
  void process();

private:
  //! type definition for database template specialisation
  typedef FeaturesDB<SURFPoint, RoverState, CvSURFPoint> FeaturesDB_t;

  //tmp
  cv_bridge::CvImagePtr cv_img_ptr_;
  geometry_msgs::PoseStamped ctrl_vec_;
  nav_msgs::Path est_traj_msg_;
  nav_msgs::Path true_traj_msg_;
  nav_msgs::Path in_traj_msg_;
  vector<CvPoint3D64f> points3d_;
  CvSeq* keypoints_ptr_;
  CvSeq* descriptors_ptr_;
  FeaturesDB_t db_; //need header
  sscrovers_pmslam_common::Map3D map3d_msg_;
  sscrovers_pmslam_common::PMSlamData pmslamdata_msg_;
  sscrovers_pmslam_common::PtPairs ptpairs_msg_;
  vector<int> ptpairs_;
  int kp_nsec, db_nsec, pt_nsec, img_nsec;
  PointCloud ptcloud_;
  ros::Publisher ptcloud_pub_;
  ros::Publisher pose_pub_;


  //! name for image window
  static const char IMAGE_WINDOW_NAME[];

  //! BGR colors definition for landmarks display - names from openoffice :)
  enum Color
  {
    Red, Orange, Yellow, Green, LightBlue, Cyan, Blue, LightMagenta, White, Pink
  };
  static const CvScalar COLORS[];

  bool first_frame_f_;

  bool traj_update_f_;
  bool db_update_f_;

  bool ptcloud_pub_f_;
  bool tf_pub_f_;
  bool pose_pub_f_;

  bool debug_disp_f_;

  //! raw input image to file save flag
  bool img_to_file_f_;
  //! save raw input image to file flag
  bool ctrlvec_to_file_f_;
  //! save estimated trajectory to file flag
  bool esttraj_to_file_f_;
  //! save raw true trajectory to file flag
  bool truetraj_to_file_f_;
  //! save raw input trajectory to file flag
  bool intraj_to_file_f_;
  //! save raw paired 3d points to file flag
  bool points3d_to_file_f_;
  //! save raw features to file flag
  bool features_to_file_f_;
  //! save raw db to file flag
  bool db_to_file_f_;
  //! save raw map3d to file flag
  bool map3d_to_file_f_;
  //! save raw pmslamdata to file flag
  bool pmslamdata_to_file_f_;
  //! save raw ptpairs to file flag
  bool ptpairs_to_file_f_;

  //! path for all data
  string data_path_;
  //! sub-directory for raw data files
  string raw_data_subdir_name_;
  //! full path for raw data files
  string raw_data_path_;
  //! sub-directory for images
  string images_subdir_name_;
  //! full path for images
  string images_path_;
  //! sub-directory for custom reports
  string reports_subdir_name_;
  //! full path for reports files
  string reports_path_;

  //! node handler for image operations
  image_transport::ImageTransport it_;

  //! image topic name parameter
  string img_sub_topic_name_;
  //! control vector topic name parameter
  string ctrlvec_sub_topic_name_;
  //! estimated trajectory topic name parameter
  string esttraj_sub_topic_name_;
  //! true trajectory topic name parameter
  string truetraj_sub_topic_name_;
  //! input trajectory topic name parameter
  string intraj_sub_topic_name_;
  //! paired 3d points topic name parameter
  string ptpairs3d_sub_topic_name_;
  //! extracted features topic name parameter
  string features_sub_topic_name_;
  //! features database topic name parameter
  string db_sub_topic_name_;
  //! map3d topic name parameter
  string map3d_sub_topic_name_;
  //! pmslam output data topic name parameter
  string pmslamdata_sub_topic_name_;
  //! point pairs topic name parameter
  string ptpairs_sub_topic_name_;

  //! image subscriber
  image_transport::Subscriber img_sub_;
  //! control vector subscriber
  ros::Subscriber ctrlvec_sub_;
  //! estimated trajectory subscriber
  ros::Subscriber esttraj_sub_;
  //! true trajectory subscriber
  ros::Subscriber truetraj_sub_;
  //! input trajectory subscriber
  ros::Subscriber intraj_sub_;
  //! 3d points with indexes to data base subscriber
  ros::Subscriber ptpairs3d_sub_;
  //! extracted features subscriber
  ros::Subscriber features_sub_;
  //! features database subscriber
  ros::Subscriber features_db_sub_;
  //! Map3d subscriber
  ros::Subscriber map3d_sub_;
  //! pmslam data subscriber
  ros::Subscriber pmslamdata_sub_;
  //! point pairs subscriber
  ros::Subscriber ptpairs_sub_;

  //! buffer for images
  vector<cv_bridge::CvImagePtr> cv_img_ptr_vec_;
  //! buffer for control vector message
  vector<geometry_msgs::PoseStamped> ctrl_vec_msg_vec_;
  //! buffer for estimated trajectory
  vector<nav_msgs::Path> est_traj_msg_vec_;
  //! buffer for true trajectory
  vector<nav_msgs::Path> true_traj_msg_vec_;
  //! buffer for true trajectory
  vector<nav_msgs::Path> in_traj_msg_vec_;
  //! buffer for points 3d
  vector<vector<CvPoint3D64f> > points3d_msg_vec_;
  //! buffer for keypoints
  vector<CvSeq*> keypoints_ptr_vec_;
  //! buffer for descriptors
  vector<CvSeq*> descriptors_ptr_vec_;
  //! buffer for received database
  vector<FeaturesDB_t> db_vec_; //need header
  //! buffer for Map3d message
  vector<sscrovers_pmslam_common::Map3D> map3d_msg_vec_;
  //! buffer for PMSlamData message
  vector<sscrovers_pmslam_common::PMSlamData> pmslamdata_msg_vec_;
  //! buffer for point pairs
  vector<sscrovers_pmslam_common::PtPairs> ptpairs_msg_vec_;

  //! callback function for map3d subscription
  void map3DCallback(const sscrovers_pmslam_common::Map3DConstPtr& msg);

  //! callback function for output pmslam data subscription
  void pmslamDataCallback(const sscrovers_pmslam_common::PMSlamDataConstPtr& msg);

  //! callback function for control vector subscription
  void ctrlvecCallback(const geometry_msgs::PoseStampedConstPtr& msg);

  //! callback function for estimated trajectory subscription
  void esttrajCallback(const nav_msgs::PathConstPtr& msg);

  //! callback function for input trajectory subscription
  void intrajCallback(const nav_msgs::PathConstPtr& msg);

  //! callback function for true trajectory subscription
  void truetrajCallback(const nav_msgs::PathConstPtr& msg);

  //! callback function for 3d points with index to pair with data base record subscription
  void ptpairs3dCallback(const sscrovers_pmslam_common::PairedPoints3DConstPtr& msg);

  //! callback function for features database subscription
  void featuresDBCallback(const sscrovers_pmslam_common::DynamicArrayConstPtr& msg);

  //! callback function for image subscription
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

  //! callback function for point pairs subscription
  void ptpairsCallback(const sscrovers_pmslam_common::PtPairsConstPtr& msg);

  //! callback function for image subscription
  void featuresCallback(const sscrovers_pmslam_common::DynamicArrayConstPtr& msg);

  //! create, setup and publish output pointcloud
  void publishPointCloud();
  //! publish current pose of observer
  void publishPose();
  //! publish transformation for current position of observer
  void publishTransformation();

  /*!
   * \brief Callback function for dynamic reconfigure server.
   *
   * This description is displayed lower in the doxygen as an extended description along with
   * the above brief description.
   * \param junk       This is a cool junk varible
   */
  //void configCallback(node_example::node_example_paramsConfig &config, uint32_t level);

nav_msgs::Path output_traj_msg;
};

#endif //DATALOGGER_CORE_H
