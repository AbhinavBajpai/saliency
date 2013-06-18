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
  private_node_handle.param("sub_traj_topic_name", sub_traj_topic_name_, string("est_traj"));
  private_node_handle.param("sub_pt_topic_name", sub_pt_topic_name_, string("ptpairs"));
  private_node_handle.param("pub_pt_topic_name", pub_pt_topic_name_, string("points3d"));
  private_node_handle.param("sub_db_topic_name", sub_db_topic_name_, string("sal_db"));
  private_node_handle.param("sub_features_topic_name", sub_features_topic_name_, string("features"));
  //! save to file flags
  private_node_handle.param("traj_to_file", traj_to_file_f_, bool(false));
  private_node_handle.param("features_to_file", features_to_file_f_, bool(false));

  //! publishers
  points3d_pub_ = _n->advertise<sscrovers_pmslam_common::PairedPoints3D>(pub_pt_topic_name_.c_str(), 1);

  //! subscribers
  ptpairs_sub_ = _n->subscribe(sub_pt_topic_name_.c_str(), 1, &DPCore::ptpairsCallBack, this);
  trajectory_sub_ = _n->subscribe(sub_traj_topic_name_.c_str(), 1, &DPCore::trajectoryCallBack, this);
  features_db_sub_ = _n->subscribe("SAL_db", 1, &DPCore::featuresDBCallBack, this);

  //pmslam functional module
  direct_depth_ptr_ = new DirectDepthDP();

  step_ = -1;
}

DPCore::~DPCore()
{
}

void DPCore::process()
{
  //put here everything that should run with node
  if (step_ >= 0)
  {
    if ((est_traj_msg_.poses.size() >= ptpairs_msg_.header.stamp.nsec) && dbTest)
    {
      double _rng = 10;
      int px = 320;
      int py = 240;
      double VFOV = 52.2;
      double HFOV = 66.7;
      double pan = 0.0;
      double tilt = 45.0;
      double CamH = 0.52;

      direct_depth_ptr_->directDepth(&sal_db, ptpairs_msg_.pairs, pt_pairs_out_, &curr_pose_, points3d_, _rng, px,
                                     py, VFOV, HFOV, pan, tilt, CamH);
	dbTest=false;
    }
  }
  publishPoints3D();
}

void DPCore::trajectoryCallBack(const nav_msgs::PathConstPtr& msg)
{
  //ROS_INFO("trajectory found");
  est_traj_msg_ = *msg;
  if (step_ >= 0)
  {
    if ((int)est_traj_msg_.poses.size() > step_)
    {
      curr_pose_.x = est_traj_msg_.poses[step_].pose.position.x;
      curr_pose_.y = est_traj_msg_.poses[step_].pose.position.y;
      curr_pose_.z = est_traj_msg_.poses[step_].pose.position.z;

      double _roll, _pitch, _yaw;
#if ROS_VERSION_MINIMUM(1, 8, 0) // if current ros version is >= 1.8.0 (fuerte)
      //min. fuerte
      tf::Quaternion _q;
      tf::quaternionMsgToTF(est_traj_msg_.poses[step_].pose.orientation, _q);
      tf::Matrix3x3(_q).getRPY(_roll, _pitch, _yaw);
#else
      //electric and older
      btQuaternion _q;
      tf::quaternionMsgToTF(est_traj_msg_.poses[step_].pose.orientation, _q);
      btMatrix3x3(_q).getRPY(_roll, _pitch, _yaw);
#endif

      curr_pose_.roll = _roll;
      curr_pose_.pitch = _pitch;
      curr_pose_.yaw = _yaw;
      //data_completed_f_ = true;
    }
    else
    {
      ROS_WARN("There is no trajectory point corresponding to features data...2");
      //data_completed_f_ = false;
    }

    
  }
}

void DPCore::ptpairsCallBack(const sscrovers_pmslam_common::PtPairsConstPtr& msg)
{
  step_ = msg->header.stamp.nsec;
  ptpairs_msg_ = *msg;
  //ROS_INFO("ptpairs found, %i", ptpairs_msg_.pairs[3]);
  dbTest=true;
}

void DPCore::featuresDBCallBack(const sscrovers_pmslam_common::SALVector& msg)
{
  //ROS_INFO("Sal_db found");
  if (msg.dims > 0)
  {
    sal_db.resize(msg.dims);
    memcpy(sal_db.data(), msg.data.data(), msg.dims * sizeof(sscrovers_pmslam_common::SPoint));
  }
  else
    ROS_ERROR("No data in database topic");

  process();

}

void DPCore::publishPoints3D()
{
  sscrovers_pmslam_common::PairedPoints3D _ppoints3d_msg;

  _ppoints3d_msg.header.stamp.nsec = step_;

  _ppoints3d_msg.pts.resize(pt_pairs_out_.size());

  for (int i = 0; i < (int)pt_pairs_out_.size(); i++)
  {
    _ppoints3d_msg.pts[i].id = pt_pairs_out_[i];
    _ppoints3d_msg.pts[i].pt.x = points3d_[i].x;
    _ppoints3d_msg.pts[i].pt.y = points3d_[i].y;
    _ppoints3d_msg.pts[i].pt.z = points3d_[i].z;
  }
  points3d_pub_.publish(_ppoints3d_msg);
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
