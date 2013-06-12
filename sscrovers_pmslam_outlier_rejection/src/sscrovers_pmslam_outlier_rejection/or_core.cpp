#include "or_core.h"

#include <iostream>
#include <sstream>
#include <fstream>
using namespace std;

ORCore::ORCore(ros::NodeHandle *_n)
{
  step_ = -1;
  curr_pose_ptr_ = new RoverState;
  filter_keypoints_ptr_ = new FilterKeypointsOR(&step_, curr_pose_ptr_, &db_);

  // Initialise node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can be run simultaneously
  // while using different parameters.
  ros::NodeHandle private_node_handle("~");
  private_node_handle.param("rate", rate_, int(10));
  // topic names
  private_node_handle.param("output_features_topic_name", sub_features_topic_name_, string("features"));
  private_node_handle.param("sub_trajectory_topic_name", sub_trajectory_topic_name_, string("est_traj"));
  private_node_handle.param("pub_ptpairs_topic_name", pub_ptpairs_topic_name_, string("ptpairs"));
  //camera params - TODO from camera info?, camera image? param server?
  private_node_handle.param("px", filter_keypoints_ptr_->px_, int(320));
  private_node_handle.param("py", filter_keypoints_ptr_->py_, int(240));
  //pmslam params
  private_node_handle.param("lm_track", filter_keypoints_ptr_->lm_track_, int(94));
  private_node_handle.param("scale", filter_keypoints_ptr_->scale_, int(1));
  // debug flags
  private_node_handle.param("features_to_file", features_to_file_f_, bool(false));
  private_node_handle.param("ptpoints_to_file", ptpoints_to_file_f_, bool(false));
  private_node_handle.param("db_to_file", db_to_file_f_, bool(false));
  private_node_handle.param("traj_to_file", traj_to_file_f_, bool(false));

  //subscribers
  features_sub_ = _n->subscribe(sub_features_topic_name_.c_str(), 1, &ORCore::featuresCallback, this);
  trajectory_sub_ = _n->subscribe(sub_trajectory_topic_name_.c_str(), 1, &ORCore::trajectoryCallback, this);

  //publishers
  ptpairs_pub_ = _n->advertise<sscrovers_pmslam_common::PtPairs>(pub_ptpairs_topic_name_.c_str(), 1);
  db_pub_ = _n->advertise<sscrovers_pmslam_common::DynamicArray>("temp_db", 1);

  data_completed_f_ = false;
}

ORCore::~ORCore()
{

}

void ORCore::process()
{
  //put here everything that should run with node loop

  //if get complet data and step is updated
  if ((data_completed_f_) && (step_ > 0))
  {
    data_completed_f_ = false;
    filter_keypoints_ptr_->filterKeypoints();
    publishPtPairs();
    publishDB(); //send data to DB
  }
}

void ORCore::sendToSurfDataBase()
{
  //publishing all db
}

void ORCore::publishPtPairs()
{
  // push forward stamp
  ptpairs_msg_.header.stamp = stamp_;
  // size of data to publish
  int size = filter_keypoints_ptr_->pt_pairs_.size();
  // reserve space for data
  ptpairs_msg_.pairs.resize(size);
  // copy data to message structure
  memcpy(ptpairs_msg_.pairs.data(), filter_keypoints_ptr_->pt_pairs_.data(), size * sizeof(int)); //change to ptpairs_msg_
  // publish msg
  ptpairs_pub_.publish(ptpairs_msg_);

}

void ORCore::featuresCallback(const sscrovers_pmslam_common::DynamicArrayFeatureConstPtr& msg)
{

  step_ = msg->header.stamp.nsec;
  stamp_ = msg->header.stamp;

  unsigned int kp_size = msg->dims[0] * msg->dims[1] * sizeof(CvSURFPoint);
  //unsigned int ds_size = msg->dims[0] * msg->dims[2] * sizeof(float);

  CvMemStorage *mem_storage_kp = cvCreateMemStorage(0);
  filter_keypoints_ptr_->keypoints_ptr_ = cvCreateSeq(0, sizeof(CvSeq), sizeof(CvSURFPoint), mem_storage_kp);
  cvSeqPushMulti(filter_keypoints_ptr_->keypoints_ptr_, (CvSURFPoint*)&msg->data[0], msg->dims[0]);

  CvMemStorage *mem_storage_dscp = cvCreateMemStorage(0);
  filter_keypoints_ptr_->descriptors_ptr_ = cvCreateSeq(0, sizeof(CvSeq), sizeof(float) * msg->dims[2],
                                                        mem_storage_dscp);

  typedef struct float64
  {
    float x[64];
  } float64;
  cvSeqPushMulti(filter_keypoints_ptr_->descriptors_ptr_, (float64*)(&msg->data[0] + kp_size), msg->dims[0]);

  process();

}

void ORCore::trajectoryCallback(const nav_msgs::PathConstPtr& msg)
{

  est_traj_msg_ = *msg;
  if (step_ >= 0)
  {
    if ((int)est_traj_msg_.poses.size() > step_)
    {
      curr_pose_ptr_->x = est_traj_msg_.poses[step_].pose.position.x;
      curr_pose_ptr_->y = est_traj_msg_.poses[step_].pose.position.y;
      curr_pose_ptr_->z = est_traj_msg_.poses[step_].pose.position.z;

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

      curr_pose_ptr_->roll = _roll;
      curr_pose_ptr_->pitch = _pitch;
      curr_pose_ptr_->yaw = _yaw;

      data_completed_f_ = true;
    }
    else
    {
      ROS_WARN("There is no trajectory point corresponding to features data...");
      data_completed_f_ = false;
    }
  }

  
}

void ORCore::publishDB()
{
  //ROS_STATIC_ASSERT(sizeof(MyVector3) == 24);

  sscrovers_pmslam_common::DynamicArray serialized_db;

  serialized_db.header.stamp.nsec = step_;

  serialized_db.dims.push_back(db_.storage_->size());
  serialized_db.dims.push_back(1);
  serialized_db.types.push_back("SURFPoint");
  serialized_db.data.resize(sizeof(SURFPoint) * db_.storage_->size());
  memcpy(serialized_db.data.data(), db_.storage_->data(), serialized_db.dims[0] * sizeof(SURFPoint));

  db_pub_.publish(serialized_db);
}

//-----------------------------MAIN-------------------------------

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "outlier_rejection");
  ros::NodeHandle n;

  // Create a new NodeExample object.
  ORCore *or_core = new ORCore(&n); // __atribute__... only for eliminate unused variable warning :)

  //until we use only img callback, below is needless
  // Tell ROS how fast to run this node.
  ros::Rate r(or_core->rate_);

  // Main loop.

  while (n.ok())
  {
    //or_core->Process();
    //fd_core->Publish();
    ros::spinOnce();
    r.sleep();
  }

  return 0;
} // end main()
