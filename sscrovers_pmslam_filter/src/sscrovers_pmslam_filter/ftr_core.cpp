#include "ftr_core.h"
#include <iostream>
#include <fstream>
#include <sstream>

using std::stringstream;
using std::endl;

//maybe to common func
string convertInt(int number)
{
  stringstream ss; //create a stringstream
  ss << number; //add number to the stream
  return ss.str(); //return a string with the contents of the stream
}

FtrCore::FtrCore(ros::NodeHandle *_n)
{
  info_filter_ptr_ = new InformationFilterFtr(&step_, &curr_pose_, &db_);

  // Initialise node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can be run simultaneously
  // while using different parameters.
  ros::NodeHandle private_node_handle("~");
  private_node_handle.param("rate", rate_, int(10));
  //! topics name
  //private_node_handle.param("pub_map3d_topic_name", pub_map3d_topic_name_, string("map3d"));
  private_node_handle.param("pub_output_data_topic_name", pub_output_data_topic_name_, string("pmslam_data"));
  private_node_handle.param("sub_ctrl_vec_topic_name", sub_ctrl_vec_topic_name_, string("ctrl_vec"));
  private_node_handle.param("sub_trajectory_topic_name", sub_trajectory_topic_name_, string("est_traj"));
  private_node_handle.param("sub_ptpairs3d_topic_name", sub_ptpairs3d_topic_name_, string("points3d"));
  private_node_handle.param("sub_db_topic_name", sub_db_topic_name_, string("temp_db"));
  //!pmslam parameters
  private_node_handle.param("lambda_size", info_filter_ptr_->lambda_size_, int(10000));
  private_node_handle.param("odom_err", info_filter_ptr_->odom_err_, double(0.1));
  private_node_handle.param("rover_state_vec_size", info_filter_ptr_->rover_state_vec_size_, int(3));
  private_node_handle.param("measuered_state_vec_size", info_filter_ptr_->measuered_state_vec_size_, int(3));
  //! to file flags
  private_node_handle.param("db_to_file", db_to_file_f_, bool(false));
  private_node_handle.param("pt3d_to_file", pt3d_to_file_f_, bool(false));

  //!subscribers
  features_db_sub_ = _n->subscribe(sub_db_topic_name_.c_str(), 1, &FtrCore::featuresDBCallBack, this);
  ctrl_vec_sub_ = _n->subscribe(sub_ctrl_vec_topic_name_.c_str(), 1, &FtrCore::ctrlvecCallBack, this);
  trajectory_sub_ = _n->subscribe(sub_trajectory_topic_name_.c_str(), 1, &FtrCore::trajectoryCallBack, this);
  ptpairs3d_sub_ = _n->subscribe(sub_ptpairs3d_topic_name_.c_str(), 1, &FtrCore::ptpairs3dCallBack, this);

  //!publishers
  //map3d_pub_ = _n->advertise<sscrovers_pmslam_common::Map3D> (pub_map3d_topic_name_.c_str(), 10);
  pmdata_pub_ = _n->advertise<sscrovers_pmslam_common::PMSlamData>(pub_output_data_topic_name_.c_str(), 10);
  db_pub_ = _n->advertise<sscrovers_pmslam_common::DynamicArray>("output_db", 10);

  step_ = -1;

  first_frame_f_ = true;
  traj_update_f_ = false;
}

FtrCore::~FtrCore()
{

}

void FtrCore::process()
{
  //put here everything that should run in node loop

  //for compatibility with pmslam code
  Mat *_u = new Mat(info_filter_ptr_->rover_state_vec_size_, 1, CV_64F);
  (*_u).at<double>(0, 0) = ctrl_vec_.pose.position.x;
  (*_u).at<double>(1, 0) = ctrl_vec_.pose.position.y;
  (*_u).at<double>(2, 0) = ctrl_vec_.pose.position.z;

  if (step_ >= 0)
  {
    if (traj_update_f_)
    {
      if (first_frame_f_)
      {
        ROS_INFO("SLAM filter initialisation...");
        info_filter_ptr_->Init(&curr_pose_);
        first_frame_f_ = false;
        traj_update_f_ = false;
      }
      else
      {
        info_filter_ptr_->CreateMap(_u, &ptpairs_, &points3d_);
      }
    }

  }
  delete _u;

  //publishMap3D();
  publishPMSlamData();
  publishDB();
}

void FtrCore::publishMap3D()
{
  map3d_msg_.header.stamp.nsec = step_;
  map3d_msg_.landmarks.resize(info_filter_ptr_->map3d_ptr_->map.size());
  for (unsigned int i = 0; i < map3d_msg_.landmarks.size(); i++)
  {
    map3d_msg_.landmarks[i].index = info_filter_ptr_->map3d_ptr_->map[i].index;
    map3d_msg_.landmarks[i].matpos = info_filter_ptr_->map3d_ptr_->map[i].matPos;
    map3d_msg_.landmarks[i].position.x = info_filter_ptr_->map3d_ptr_->map[i].position.x;
    map3d_msg_.landmarks[i].position.y = info_filter_ptr_->map3d_ptr_->map[i].position.y;
    map3d_msg_.landmarks[i].position.z = info_filter_ptr_->map3d_ptr_->map[i].position.z;
    map3d_msg_.landmarks[i].stddev = info_filter_ptr_->map3d_ptr_->map[i].stddev;
  }
  map3d_pub_.publish(map3d_msg_);
}

void FtrCore::publishPMSlamData()
{
  pmslam_data_msg_.header.stamp.nsec = step_;
  pmslam_data_msg_.map_out.resize(info_filter_ptr_->PMSLAM_Data_msg.MapOut.positions.x.size());
  for (unsigned int i = 0; i < pmslam_data_msg_.map_out.size(); i++)
  {
    pmslam_data_msg_.map_out[i].pt.x = info_filter_ptr_->PMSLAM_Data_msg.MapOut.positions.x[i];
    pmslam_data_msg_.map_out[i].pt.y = info_filter_ptr_->PMSLAM_Data_msg.MapOut.positions.y[i];
    pmslam_data_msg_.map_out[i].pt.z = info_filter_ptr_->PMSLAM_Data_msg.MapOut.positions.z[i];
    pmslam_data_msg_.map_out[i].id = info_filter_ptr_->PMSLAM_Data_msg.MapOut.ID[i];
  }
  pmslam_data_msg_.trajectory_out.position.x = info_filter_ptr_->PMSLAM_Data_msg.TrajectoryOut.x;
  pmslam_data_msg_.trajectory_out.position.y = info_filter_ptr_->PMSLAM_Data_msg.TrajectoryOut.y;
  pmslam_data_msg_.trajectory_out.position.z = info_filter_ptr_->PMSLAM_Data_msg.TrajectoryOut.z;
  pmdata_pub_.publish(pmslam_data_msg_);
}

void FtrCore::ctrlvecCallBack(const geometry_msgs::PoseStampedConstPtr& msg)
{
  ctrl_vec_ = *msg;
}

void FtrCore::trajectoryCallBack(const nav_msgs::PathConstPtr& msg)
{
  est_traj_ = *msg;
  if (step_ >= 0)
  {
    if ((int)est_traj_.poses.size() > step_)
    {
      curr_pose_.x = est_traj_.poses[step_].pose.position.x;
      curr_pose_.y = est_traj_.poses[step_].pose.position.y;
      curr_pose_.z = est_traj_.poses[step_].pose.position.z;

      double _roll, _pitch, _yaw;
#if ROS_VERSION_MINIMUM(1, 8, 0) // if current ros version is >= 1.8.0 (fuerte)
      //min. fuerte
      tf::Quaternion _q;
      tf::quaternionMsgToTF(est_traj_.poses[step_].pose.orientation, _q);
      tf::Matrix3x3(_q).getRPY(_roll, _pitch, _yaw);
#else
      //electric and older
      btQuaternion _q;
      tf::quaternionMsgToTF(est_traj_.poses[step_].pose.orientation, _q);
      btMatrix3x3(_q).getRPY(_roll, _pitch, _yaw);
#endif

      curr_pose_.roll = _roll;
      curr_pose_.pitch = _pitch;
      curr_pose_.yaw = _yaw;

      //data_completed_f_ = true;
    }
    else
    {
      ROS_WARN("There is no trajectory point corresponding to features data...");
      //data_completed_f_ = false;
    }
  }
  traj_update_f_ = true;
}

void FtrCore::ptpairs3dCallBack(const sscrovers_pmslam_common::PairedPoints3DConstPtr& msg)
{
  step_ = msg->header.stamp.nsec;
  unsigned int in_size = msg->pts.size();
  ptpairs_.resize(in_size);
  points3d_.resize(in_size);
  for (unsigned int i = 0; i < in_size; i++)
  {
    ptpairs_[i] = msg->pts[i].id;
    points3d_[i].x = msg->pts[i].pt.x;
    points3d_[i].y = msg->pts[i].pt.y;
    points3d_[i].z = msg->pts[i].pt.z;
  }
}

void FtrCore::featuresDBCallBack(const sscrovers_pmslam_common::DynamicArrayConstPtr& msg)
{

  if (msg->dims[0] > 0)
  {
    db_.storage_->resize(msg->dims[0]);
    memcpy(db_.storage_->data(), msg->data.data(), msg->dims[0] * sizeof(SURFPoint));
  }
  else
    ROS_ERROR("No data in database topic");

  bool rawdb_to_file_f_ = false;

}

void FtrCore::publishDB()
{
 

  //ROS_STATIC_ASSERT(sizeof(MyVector3) == 24);

  sscrovers_pmslam_common::DynamicArray serialized_db;

  serialized_db.header.stamp.nsec = step_;

  serialized_db.dims.push_back(db_.storage_->size());
  serialized_db.dims.push_back(1);
  serialized_db.types.push_back("SURFPoint");
  serialized_db.data.resize(sizeof(SURFPoint) * db_.storage_->size());
  memcpy(serialized_db.data.data(), db_.storage_->data(), serialized_db.dims[0] * sizeof(SURFPoint));

  bool rawdb_to_file_f_ = false;
  

  db_pub_.publish(serialized_db);
}

/*
 void FtrCore::configCallback(node_example::node_example_paramsConfig &config, uint32_t level)
 {
 // Set class variables to new values. They should match what is input at the dynamic reconfigure GUI.
 message = config.message.c_str();
 a = config.a;
 b = config.b;
 } // end configCallback()
 */

//-----------------------------MAIN----------------------------
int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "slam_filter");
  ros::NodeHandle n;

  // Create a new NodeExample object.
  FtrCore *ftr_core = new FtrCore(&n);

  // Tell ROS how fast to run this node.
  ros::Rate r(ftr_core->rate_);

  // Main loop.

  while (n.ok())
  {
    ftr_core->process();
    //fd_core->Publish();
    ros::spinOnce();
    r.sleep();
  }

  //ros::spin();

  return 0;
} // end main()
