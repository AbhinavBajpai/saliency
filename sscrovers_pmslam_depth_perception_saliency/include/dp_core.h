#ifndef DP_CORE_H
#define DP_CORE_H

// ROS includes
#include "ros/ros.h"
#include "ros/time.h"
#include "tf/tf.h"

//OpenCV
#include <cv.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

//ROS messages
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Path.h>
#include "sscrovers_pmslam_common/SALVector.h"
#include "sscrovers_pmslam_common/SPoint.h"
#include "sscrovers_pmslam_common/PairedPoints3D.h"
#include "sscrovers_pmslam_common/PtPairs.h"
#include "sscrovers_pmslam_common/DynamicArray.h"

//pmslam modules
#include "direct_depth_dp.h"
#include "triangulate_dp.h"

#include "RoverState.h"
#include "ptpairs.h"


#include "features_db.h"

using std::string;

class DPCore
{
public:
  //! Constructor.
  DPCore(ros::NodeHandle *_n);

  //! Destructor.
  ~DPCore();

  //! rate for node main loop
  int rate_;

  //!Everything in this function is processed in node loop
  void process();
  std::vector <sscrovers_pmslam_common::SPoint> sal_db;
  bool dbTest;
private:
  //! save to file flags
  bool traj_to_file_f_;
  bool features_to_file_f_;

  //! topic names
  string pub_pt_topic_name_, sub_pt_topic_name_, sub_traj_topic_name_, sub_db_topic_name_, sub_features_topic_name_;

  //! publisher
  ros::Publisher points3d_pub_;

  //! subscribers
  ros::Subscriber ptpairs_sub_, trajectory_sub_, features_db_sub_, features_sub_;

  //! point pairs message
  sscrovers_pmslam_common::PtPairs ptpairs_msg_;

  //! output 3d point pairs message - temporary only for compatibility with pmslam - should be changed to paired3dpoints msg
  vector<int> pt_pairs_out_;
  vector<CvPoint3D64f> points3d_;
  //OR
  //sscrovers_pmslam_common::PairedPoints3D ppoints3d_msg_;

  //! estimatet trajectory
  nav_msgs::Path est_traj_msg_;

  //! current pose
  RoverState curr_pose_;

  //! current step
  int step_;

  //! Object of functional class
  DirectDepthDP* direct_depth_ptr_;

  //! Function for publish 3D points data with db pair info
  void publishPoints3D();

  //! Callback function for point pairs subscription
  void ptpairsCallBack(const sscrovers_pmslam_common::PtPairsConstPtr& msg);

  //! Callback function for rover trajectory subscription.
  void trajectoryCallBack(const nav_msgs::PathConstPtr& msg);

  //! Callback function for features database subscription.
  void featuresDBCallBack(const sscrovers_pmslam_common::SALVector& msg);


};

#endif //DP_CORE_H
