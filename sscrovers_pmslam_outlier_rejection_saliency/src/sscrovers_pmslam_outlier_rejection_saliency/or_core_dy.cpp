#include "or_core_dy.h"

#include <iostream>
#include <sstream>
#include <fstream>
using namespace std;

void trajectoryCallbackDy(const nav_msgs::PathConstPtr& msg, ORCore * oc);
void featuresCallbackDy(const sscrovers_pmslam_common::DynamicArrayConstPtr& msg, ORCore * oc);


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

  // publishers
  trajectory_ds = dySubscriber<nav_msgs::Path>("est_traj", *_n);
  features_ds = dySubscriber<sscrovers_pmslam_common::DynamicArray>("features", *_n);

  //subscribers
  ptpairs_dp = dyPublisher<sscrovers_pmslam_common::PtPairs>("ptpairs", *_n);
  db_dp = dyPublisher<sscrovers_pmslam_common::DynamicArray>("temp_db", *_n);

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
  // store to file
  if (ptpoints_to_file_f_)
  {
    //store keypoints to file
    stringstream _a;
    _a << step_;
    string filename = "/home/sscrovers/pm_output/or_ptpoints_" + _a.str() + ".log";
    std::ofstream file;
    file.open(filename.c_str());
    //put if here!!
    file << "total: " << filter_keypoints_ptr_->pt_pairs_.size() << endl;
    file << "#\tid" << endl;
    for (unsigned int i = 0; i < filter_keypoints_ptr_->pt_pairs_.size(); i++)
    {
      file << i << ": \t" << filter_keypoints_ptr_->pt_pairs_[i] << endl;
    }
    file.close();
  }

  // push forward stamp
  ptpairs_msg_.header.stamp = stamp_;
  // size of data to publish
  int size = filter_keypoints_ptr_->pt_pairs_.size();
  // reserve space for data
  ptpairs_msg_.pairs.resize(size);
  // copy data to message structure
  memcpy(ptpairs_msg_.pairs.data(), filter_keypoints_ptr_->pt_pairs_.data(), size * sizeof(int)); //change to ptpairs_msg_

  // publish msg
  ptpairs_dp.publish(ptpairs_msg_);

}

void ORCore::featuresCallback(const sscrovers_pmslam_common::DynamicArrayConstPtr& msg)
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

  if (features_to_file_f_)
  {
    //store keypoints to file
    stringstream _a1;
    _a1 << step_;
    string filename1 = "/home/sscrovers/pm_output/or_keypoints_" + _a1.str() + ".log";
    std::ofstream file1;
    file1.open(filename1.c_str());
    //put if here!!
    file1 << "total: " << filter_keypoints_ptr_->keypoints_ptr_->total << endl;
    file1 << "#\tpt.x\tpt.y\tlap\tsize\tdir\thess" << endl;
    for (int i = 0; i < filter_keypoints_ptr_->keypoints_ptr_->total; i++)
    {
      CvSURFPoint * r = (CvSURFPoint*)cvGetSeqElem(filter_keypoints_ptr_->keypoints_ptr_, i);
      file1 << i << ": \t" << r->pt.x << "\t" << r->pt.y << "\t" << r->laplacian << "\t" << r->size << "\t" << r->dir
          << "\t" << r->hessian << "\t" << endl;
    }
    file1.close();
  }

  if (features_to_file_f_)
  {
    //store descriptors to file
    stringstream _a;
    _a << step_;
    string filename = "/home/sscrovers/pm_output/or_descriptors_" + _a.str() + ".log";
    std::ofstream file;
    file.open(filename.c_str());
    //put if here!!
    file << "total: " << filter_keypoints_ptr_->descriptors_ptr_->total << endl;
    file << "#\tpt.x\tpt.y\tlap\tsize\tdir\thess" << endl;
    int _s = filter_keypoints_ptr_->descriptors_ptr_->elem_size / sizeof(float);
    for (int i = 0; i < filter_keypoints_ptr_->descriptors_ptr_->total; i++)
    {
      float* r = (float*)cvGetSeqElem(filter_keypoints_ptr_->descriptors_ptr_, i);
      file << i << ": ";
      for (int j = 0; j < _s; j++)
      {
        file << "\t" << r[j];
      }
      file << endl;
    }
    file.close();
  }

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

  if (traj_to_file_f_)
  {
    //!!!split this to function for trajectory and point!!!
    //store trajectory to file
    stringstream _a;
    _a << step_;
    string filename = "/home/sscrovers/pm_output/or_esttraj_" + _a.str() + ".log";
    std::ofstream file;
    file.open(filename.c_str());
    //put if here!!
    file << "total: " << est_traj_msg_.poses.size() << endl;
    file << "current pt: #" << step_ << "\tx=" << curr_pose_ptr_->x << "\ty=" << curr_pose_ptr_->y << "\tz="
        << curr_pose_ptr_->z << "\tr=" << curr_pose_ptr_->roll << "\tp=" << curr_pose_ptr_->pitch << "\ty="
        << curr_pose_ptr_->yaw << endl << endl;
    file << "#\tx\ty\tz\tq1\tq2\tq3\tq4\t\troll\tpitch\tyaw" << endl;
    for (unsigned int i = 0; i < est_traj_msg_.poses.size(); i++)
    {
      double _roll, _pitch, _yaw;
#if ROS_VERSION_MINIMUM(1, 8, 0) // if current ros version is >= 1.8.0 (fuerte)
      //min. fuerte
      tf::Quaternion _q;
      tf::quaternionMsgToTF(est_traj_msg_.poses[i].pose.orientation, _q);
      tf::Matrix3x3(_q).getRPY(_roll, _pitch, _yaw);
#else
      //electric and older
      btQuaternion _q;
      tf::quaternionMsgToTF(est_traj_msg_.poses[i].pose.orientation, _q);
      btMatrix3x3(_q).getRPY(_roll, _pitch, _yaw);
#endif

      file << i << ": " << est_traj_msg_.poses[i].pose.position.x << "\t" << est_traj_msg_.poses[i].pose.position.y << "\t"
          << est_traj_msg_.poses[i].pose.position.z << "\t" << est_traj_msg_.poses[i].pose.orientation.x << "\t"
          << est_traj_msg_.poses[i].pose.orientation.y << "\t" << est_traj_msg_.poses[i].pose.orientation.z << "\t"
          << est_traj_msg_.poses[i].pose.orientation.w << "\t" << _roll << "\t" << _pitch << "\t" << _yaw << endl;
    }
    file.close();
  }
}

void ORCore::publishDB()
{
  if (db_to_file_f_)
  {
    //store descriptors to file
    stringstream _a;
    _a << step_;
    string filename = "or_db_" + _a.str() + ".log";
    std::ofstream file;
    file.open(filename.c_str());
    //put if here!!
    file << "total: " << db_.storage_->size() << endl;
    file << "#\t index\n" << "pt.pt.x\t pt.pt.y\t lap\t size\t dir\t hess\n"
        << "state.x\t state.y\t state.z\t state.yaw\t state.pitch\t state.roll\n" << "pxLocation[0]\t pxLocation[1]\n"
        << "gtruth[0]\t gtruth[1]\t gtruth[2]\n" << "n\t id\t step\t flag\n" << "desc[64]" << endl << endl;
    for (unsigned int i = 0; i < db_.storage_->size(); i++)
    {
      file << i << ": " << (*db_.storage_)[i].index << endl << (*db_.storage_)[i].pt.pt.x << "\t"
          << (*db_.storage_)[i].pt.pt.y << "\t" << (*db_.storage_)[i].pt.laplacian << "\t" << (*db_.storage_)[i].pt.size
          << "\t" << (*db_.storage_)[i].pt.dir << "\t" << (*db_.storage_)[i].pt.hessian << endl
          << (*db_.storage_)[i].state.x << "\t" << (*db_.storage_)[i].state.y << "\t" << (*db_.storage_)[i].state.z
          << "\t" << (*db_.storage_)[i].state.yaw << "\t" << (*db_.storage_)[i].state.pitch << "\t"
          << (*db_.storage_)[i].state.roll << endl << (*db_.storage_)[i].pxLocation[0] << "\t"
          << (*db_.storage_)[i].pxLocation[1] << endl << (*db_.storage_)[i].gtruth[0] << "\t"
          << (*db_.storage_)[i].gtruth[1] << "\t" << (*db_.storage_)[i].gtruth[2] << endl << "\t"
          << (*db_.storage_)[i].n << "\t" << (*db_.storage_)[i].id << "\t" << (*db_.storage_)[i].step << "\t"
          << (*db_.storage_)[i].flag << endl;

      for (int j = 0; j < 64; j++)
      {
        file << (*db_.storage_)[i].descriptor[j] << "\t";
      }
      file << endl << endl;
    }
    file.close();
  }

  //ROS_STATIC_ASSERT(sizeof(MyVector3) == 24);

  sscrovers_pmslam_common::DynamicArray serialized_db;

  serialized_db.header.stamp.nsec = step_;

  serialized_db.dims.push_back(db_.storage_->size());
  serialized_db.dims.push_back(1);
  serialized_db.types.push_back("SURFPoint");
  serialized_db.data.resize(sizeof(SURFPoint) * db_.storage_->size());
  memcpy(serialized_db.data.data(), db_.storage_->data(), serialized_db.dims[0] * sizeof(SURFPoint));

  bool rawdb_to_file_f_ = false;
  if (rawdb_to_file_f_)
  {
    //store descriptors to file
    stringstream _a;
    _a << step_;
    string filename = "/home/sscroversovers/pm_output/or_rawdb_" + _a.str() + "";
    std::ofstream file;
    file.open(filename.c_str());
    //put if here!!
    file << "rawtotal: " << serialized_db.dims[0] * sizeof(SURFPoint) << endl;
    file << "total: " << serialized_db.data.size() << endl;
    for (unsigned int i = 0; i < serialized_db.data.size(); i++)
    {
      file << serialized_db.data[i] << endl;
    }
    file.close();
  }

  db_dp.publish(serialized_db);
}


void featuresCallbackDy(const sscrovers_pmslam_common::DynamicArrayConstPtr& msg, ORCore * oc)
{

  oc->step_ = msg->header.stamp.nsec;
  oc->stamp_ = msg->header.stamp;

  unsigned int kp_size = msg->dims[0] * msg->dims[1] * sizeof(CvSURFPoint);
  //unsigned int ds_size = msg->dims[0] * msg->dims[2] * sizeof(float);

  CvMemStorage *mem_storage_kp = cvCreateMemStorage(0);
  oc->filter_keypoints_ptr_->keypoints_ptr_ = cvCreateSeq(0, sizeof(CvSeq), sizeof(CvSURFPoint), mem_storage_kp);
  cvSeqPushMulti(oc->filter_keypoints_ptr_->keypoints_ptr_, (CvSURFPoint*)&msg->data[0], msg->dims[0]);

  CvMemStorage *mem_storage_dscp = cvCreateMemStorage(0);
  oc->filter_keypoints_ptr_->descriptors_ptr_ = cvCreateSeq(0, sizeof(CvSeq), sizeof(float) * msg->dims[2],
                                                        mem_storage_dscp);

  typedef struct float64
  {
    float x[64];
  } float64;
  cvSeqPushMulti(oc->filter_keypoints_ptr_->descriptors_ptr_, (float64*)(&msg->data[0] + kp_size), msg->dims[0]);

  if (oc->features_to_file_f_)
  {
    //store keypoints to file
    stringstream _a1;
    _a1 << oc->step_;
    string filename1 = "/home/sscrovers/pm_output/or_keypoints_" + _a1.str() + ".log";
    std::ofstream file1;
    file1.open(filename1.c_str());
    //put if here!!
    file1 << "total: " << oc->filter_keypoints_ptr_->keypoints_ptr_->total << endl;
    file1 << "#\tpt.x\tpt.y\tlap\tsize\tdir\thess" << endl;
    for (int i = 0; i < oc->filter_keypoints_ptr_->keypoints_ptr_->total; i++)
    {
      CvSURFPoint * r = (CvSURFPoint*)cvGetSeqElem(oc->filter_keypoints_ptr_->keypoints_ptr_, i);
      file1 << i << ": \t" << r->pt.x << "\t" << r->pt.y << "\t" << r->laplacian << "\t" << r->size << "\t" << r->dir
          << "\t" << r->hessian << "\t" << endl;
    }
    file1.close();
  }

  if (oc->features_to_file_f_)
  {
    //store descriptors to file
    stringstream _a;
    _a << oc->step_;
    string filename = "/home/sscrovers/pm_output/or_descriptors_" + _a.str() + ".log";
    std::ofstream file;
    file.open(filename.c_str());
    //put if here!!
    file << "total: " << oc->filter_keypoints_ptr_->descriptors_ptr_->total << endl;
    file << "#\tpt.x\tpt.y\tlap\tsize\tdir\thess" << endl;
    int _s = oc->filter_keypoints_ptr_->descriptors_ptr_->elem_size / sizeof(float);
    for (int i = 0; i < oc->filter_keypoints_ptr_->descriptors_ptr_->total; i++)
    {
      float* r = (float*)cvGetSeqElem(oc->filter_keypoints_ptr_->descriptors_ptr_, i);
      file << i << ": ";
      for (int j = 0; j < _s; j++)
      {
        file << "\t" << r[j];
      }
      file << endl;
    }
    file.close();
  }
}

void trajectoryCallbackDy(const nav_msgs::PathConstPtr& msg, ORCore * oc)
{

  oc->est_traj_msg_ = *msg;
  if (oc->step_ >= 0)
  {
    if ((int)oc->est_traj_msg_.poses.size() > oc->step_)
    {
      oc->curr_pose_ptr_->x = oc->est_traj_msg_.poses[oc->step_].pose.position.x;
      oc->curr_pose_ptr_->y = oc->est_traj_msg_.poses[oc->step_].pose.position.y;
      oc->curr_pose_ptr_->z = oc->est_traj_msg_.poses[oc->step_].pose.position.z;

      double _roll, _pitch, _yaw;
#if ROS_VERSION_MINIMUM(1, 8, 0) // if current ros version is >= 1.8.0 (fuerte)
      //min. fuerte
      tf::Quaternion _q;
      tf::quaternionMsgToTF(oc->est_traj_msg_.poses[oc->step_].pose.orientation, _q);
      tf::Matrix3x3(_q).getRPY(_roll, _pitch, _yaw);
#else
      //electric and older
      btQuaternion _q;
      tf::quaternionMsgToTF(oc->est_traj_msg_.poses[oc->step_].pose.orientation, _q);
      btMatrix3x3(_q).getRPY(_roll, _pitch, _yaw);
#endif

      oc->curr_pose_ptr_->roll = _roll;
      oc->curr_pose_ptr_->pitch = _pitch;
      oc->curr_pose_ptr_->yaw = _yaw;

      oc->data_completed_f_ = true;
    }
    else
    {
      ROS_WARN("There is no trajectory point corresponding to features data...");
      oc->data_completed_f_ = false;
    }
  }

  if (oc->traj_to_file_f_)
  {
    //!!!split this to function for trajectory and point!!!
    //store trajectory to file
    stringstream _a;
    _a << oc->step_;
    string filename = "/home/sscrovers/pm_output/or_esttraj_" + _a.str() + ".log";
    std::ofstream file;
    file.open(filename.c_str());
    //put if here!!
    file << "total: " << oc->est_traj_msg_.poses.size() << endl;
    file << "current pt: #" << oc->step_ << "\tx=" << oc->curr_pose_ptr_->x << "\ty=" << oc->curr_pose_ptr_->y << "\tz="
        << oc->curr_pose_ptr_->z << "\tr=" << oc->curr_pose_ptr_->roll << "\tp=" << oc->curr_pose_ptr_->pitch << "\ty="
        << oc->curr_pose_ptr_->yaw << endl << endl;
    file << "#\tx\ty\tz\tq1\tq2\tq3\tq4\t\troll\tpitch\tyaw" << endl;
    for (unsigned int i = 0; i < oc->est_traj_msg_.poses.size(); i++)
    {
      double _roll, _pitch, _yaw;
#if ROS_VERSION_MINIMUM(1, 8, 0) // if current ros version is >= 1.8.0 (fuerte)
      //min. fuerte
      tf::Quaternion _q;
      tf::quaternionMsgToTF(oc->est_traj_msg_.poses[i].pose.orientation, _q);
      tf::Matrix3x3(_q).getRPY(_roll, _pitch, _yaw);
#else
      //electric and older
      btQuaternion _q;
      tf::quaternionMsgToTF(est_traj_msg_.poses[i].pose.orientation, _q);
      btMatrix3x3(_q).getRPY(_roll, _pitch, _yaw);
#endif

      file << i << ": " << oc->est_traj_msg_.poses[i].pose.position.x << "\t" << oc->est_traj_msg_.poses[i].pose.position.y << "\t"
          << oc->est_traj_msg_.poses[i].pose.position.z << "\t" << oc->est_traj_msg_.poses[i].pose.orientation.x << "\t"
          << oc->est_traj_msg_.poses[i].pose.orientation.y << "\t" << oc->est_traj_msg_.poses[i].pose.orientation.z << "\t"
          << oc->est_traj_msg_.poses[i].pose.orientation.w << "\t" << _roll << "\t" << _pitch << "\t" << _yaw << endl;
    }
    file.close();
  }
}

bool ORCore::waitForData()
{
  trajectory_ds.subscribeTillOnce(boost::bind(trajectoryCallbackDy,_1,this));
  features_ds.subscribeTillOnce(boost::bind(featuresCallbackDy,_1,this));	
  return true;
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

  while (n.ok() && or_core->waitForData())
  {
    or_core->process();
    ros::spinOnce();
    r.sleep();
  }

  or_core->trajectory_ds.onShutdown();
  or_core->features_ds.onShutdown();

  or_core->ptpairs_dp.onShutdown();
  or_core->db_dp.onShutdown();

  return 0;
} // end main()
