#include "datalogger_core.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <boost/filesystem.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"

using std::stringstream;
using std::endl;

//maybe to common func
string convertInt(int number)
{
  stringstream ss; //create a stringstream
  ss << number; //add number to the stream
  return ss.str(); //return a string with the contents of the stream
}

string makeRawDataFileName(string data_name, string prefix, string postfix)
{
  return "ftr_rawdb_"; // + _a.str() + "";
}

void uint8VectorToFile(string path, string file_name, vector<uint8_t> data, string comment)
{
  /*
   stringstream _a;
   _a << msg->header.stamp.nsec;

   string filepath = path;
   if ((path[path.size() - 1] != '/') && (file_name[0] != '/'))
   filepath += "/";
   filepath += file_name;
   std::ofstream file;
   file.open(filename.c_str());
   //put if here!!
   file << "rawtotal: " << msg->dims[0] * sizeof(SURFPoint) << endl;
   file << "total: " << msg->data.size() << endl;
   for (int i = 0; i < msg->data.size(); i++)
   {
   file << msg->data[i] << endl;
   }
   file.close();
   */
}

string stringFromCurrentTime()
{
  boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
  std::stringstream ss;
  ss << now.date().year();
  ss << std::setw(2) << std::setfill('0') << static_cast<int>(now.date().month());
  ss << std::setw(2) << std::setfill('0') << now.date().day();
  ss << std::setw(2) << std::setfill('0') << now.time_of_day().hours();
  ss << std::setw(2) << std::setfill('0') << now.time_of_day().minutes();
  ss << std::setw(2) << std::setfill('0') << now.time_of_day().seconds();
  return ss.str();
}

const char DataloggerCore::IMAGE_WINDOW_NAME[] = "Image with landmarks";

const CvScalar DataloggerCore::COLORS[] = { { {0, 0, 255}}, { {0, 128, 255}}, { {0, 255, 255}}, { {0, 255, 0}}, { {255,
                                                                                                                   128,
                                                                                                                   0}},
                                           { {255, 255, 0}}, { {255, 0, 0}}, { {255, 0, 255}}, { {255, 255, 255}}, { {
                                               127, 0, 255}}};

DataloggerCore::DataloggerCore(ros::NodeHandle *_n) :
    it_(*_n)
{
  //info_filter_ptr_ = new InformationFilterFtr(&step_, &curr_pose_, &db_);

  // Initialise node parameters from launch file or command line.
  // Use a private node handle so that multiple instances of the node can be run simultaneously
  // while using different parameters.
  ros::NodeHandle private_node_handle("~");
  private_node_handle.param("rate", rate_, int(10));
  //flags
  bool no_files_f;
  //private_node_handle.param("map3d_to_file", map3d_to_file_f_, bool(false));
  private_node_handle.param("pmslamdata_to_file", pmslamdata_to_file_f_, bool(false));
  private_node_handle.param("ctrlvec_to_file", ctrlvec_to_file_f_, bool(false));
  private_node_handle.param("esttraj_to_file", esttraj_to_file_f_, bool(false));
  private_node_handle.param("intraj_to_file", intraj_to_file_f_, bool(false));
  private_node_handle.param("truetraj_to_file", truetraj_to_file_f_, bool(false));
  private_node_handle.param("ptpairs_to_file", ptpairs_to_file_f_, bool(false));
  private_node_handle.param("points3d_to_file", points3d_to_file_f_, bool(false));
  private_node_handle.param("features_to_file", features_to_file_f_, bool(false));
  private_node_handle.param("db_to_file", db_to_file_f_, bool(false));
  private_node_handle.param("img_to_file", img_to_file_f_, bool(false));
  private_node_handle.param("view_output", debug_disp_f_, bool(true));
  private_node_handle.param("publish_ptcloud", ptcloud_pub_f_, bool(true));
  private_node_handle.param("publish_tf", tf_pub_f_, bool(true));
  private_node_handle.param("publish_pose", pose_pub_f_, bool(true));
  private_node_handle.param("no_files", no_files_f, bool(true));
  //topic names
  //private_node_handle.param("map3d_sub_topic_name", map3d_sub_topic_name_, string("map3d"));
  private_node_handle.param("pmslamdata_sub_data_topic_name", pmslamdata_sub_topic_name_, string("pmslam_data"));
  private_node_handle.param("ctrlvec_sub_topic_name", ctrlvec_sub_topic_name_, string("ctrl_vec"));
  private_node_handle.param("esttraj_sub_topic_name", esttraj_sub_topic_name_, string("est_traj"));
  private_node_handle.param("intraj_sub_topic_name", intraj_sub_topic_name_, string("in_traj"));
  private_node_handle.param("truetraj_sub_topic_name", truetraj_sub_topic_name_, string("true_traj"));
  private_node_handle.param("ptpairs3d_sub_topic_name", ptpairs3d_sub_topic_name_, string("points3d"));
  private_node_handle.param("db_sub_topic_name", db_sub_topic_name_, string("output_db"));
  private_node_handle.param("features_sub_topic_name", features_sub_topic_name_, string("features"));
  private_node_handle.param("ptpairs_sub_topic_name", ptpairs_sub_topic_name_, string("ptpairs"));
  private_node_handle.param("image_sub_topic_name", img_sub_topic_name_, string("org_image"));
  //config
  string _tmpname = ros::package::getPath("sscrovers_pmslam_datalogger");
  private_node_handle.param("data_storing_path", data_path_, (_tmpname + "/output"));
  private_node_handle.param("raw_data_subdir_name", raw_data_subdir_name_, string("raw_data"));
  private_node_handle.param("images_subdir_path", images_subdir_name_, string("images"));
  private_node_handle.param("reports_subdir_path", reports_subdir_name_, string("reports"));

  if (data_path_[data_path_.size() - 1] != '/')
    data_path_ += "/";

  raw_data_path_ = data_path_ + stringFromCurrentTime() + "/" + raw_data_subdir_name_ + "/";
  images_path_ = data_path_ + stringFromCurrentTime() + "/" + images_subdir_name_ + "/";
  reports_path_ = data_path_ + stringFromCurrentTime() + "/" + reports_subdir_name_ + "/";
  boost::filesystem::create_directories(boost::filesystem::path(raw_data_path_.c_str()));
  boost::filesystem::create_directories(boost::filesystem::path(images_path_.c_str()));
  boost::filesystem::create_directories(boost::filesystem::path(reports_path_.c_str()));

  ROS_INFO("Do not forget to remove the old files from %s", data_path_.c_str());

  debug_disp_f_ = true;
  ptcloud_pub_f_ = true;
  tf_pub_f_ = false;
  pose_pub_f_ = false;

  if (no_files_f)
  {
    img_to_file_f_ = ctrlvec_to_file_f_ = esttraj_to_file_f_ = truetraj_to_file_f_ = intraj_to_file_f_ =
        points3d_to_file_f_ = features_to_file_f_ = db_to_file_f_ = pmslamdata_to_file_f_ = ptpairs_to_file_f_ = false; //= map3d_to_file_f_
  }
  //store config parameters
  //private_node_handle.param("lambda_size", info_filter_ptr_->lambda_size_, int(10000));
  //private_node_handle.param("odom_err", info_filter_ptr_->odom_err_, double(0.1));
  //private_node_handle.param("rover_state_vec_size", info_filter_ptr_->rover_state_vec_size_, int(3));
  //private_node_handle.param("measuered_state_vec_size", info_filter_ptr_->measuered_state_vec_size_, int(3));

  features_db_sub_ = _n->subscribe(db_sub_topic_name_.c_str(), 1, &DataloggerCore::featuresDBCallback, this);
  //map3d_sub_ = _n->subscribe(map3d_sub_topic_name_.c_str(), 1, &DataloggerCore::map3DCallback, this);
  pmslamdata_sub_ = _n->subscribe(pmslamdata_sub_topic_name_.c_str(), 1, &DataloggerCore::pmslamDataCallback, this);
  ctrlvec_sub_ = _n->subscribe(ctrlvec_sub_topic_name_.c_str(), 1, &DataloggerCore::ctrlvecCallback, this);
  esttraj_sub_ = _n->subscribe(esttraj_sub_topic_name_.c_str(), 1, &DataloggerCore::esttrajCallback, this);
  intraj_sub_ = _n->subscribe(intraj_sub_topic_name_.c_str(), 1, &DataloggerCore::intrajCallback, this);
  truetraj_sub_ = _n->subscribe(truetraj_sub_topic_name_.c_str(), 1, &DataloggerCore::truetrajCallback, this);
  ptpairs3d_sub_ = _n->subscribe(ptpairs3d_sub_topic_name_.c_str(), 1, &DataloggerCore::ptpairs3dCallback, this);
  features_sub_ = _n->subscribe(features_sub_topic_name_.c_str(), 1, &DataloggerCore::featuresCallback, this);
  ptpairs_sub_ = _n->subscribe(ptpairs_sub_topic_name_.c_str(), 1, &DataloggerCore::ptpairsCallback, this);
  img_sub_ = it_.subscribe(img_sub_topic_name_.c_str(), 0, &DataloggerCore::imageCallback, this);

  ptcloud_pub_ = _n->advertise<PointCloud>("ptcloud", 1);
  pose_pub_ = _n->advertise<geometry_msgs::PoseStamped>("out_pose", 1);

  step_ = -1;
  first_frame_f_ = true;

  traj_update_f_ = false;
  db_update_f_ = false;

  cv_img_ptr_.reset(new cv_bridge::CvImage);
  cv::namedWindow(IMAGE_WINDOW_NAME);

  img_nsec = pt_nsec = kp_nsec = db_nsec = 0;
}

DataloggerCore::~DataloggerCore()
{
  cv::destroyWindow(IMAGE_WINDOW_NAME);
}

void DataloggerCore::process()
{
  if (step_ >= 0)
  {
    /*
     if(ptpairs_.size()==ptpairs_msg_.pairs.size())
     for(int i =0;i<ptpairs_.size();i++)
     {
     if(ptpairs_[i]!=ptpairs_msg_.pairs[i])
     ROS_ERROR("NOT EQUAL...");
     }
     else
     ROS_ERROR("WTFF...");
     */

  }

  ROS_INFO("DataLogger steps:\n img:\t%d\nkp:\t%d\nptp:\t%d\ndb:\t%d\n", img_nsec, kp_nsec, pt_nsec, db_nsec);
  if ((step_ >= 0) && (img_nsec == kp_nsec) && (img_nsec == pt_nsec) && (db_update_f_)) // && (img_nsec == db_nsec))
    if (debug_disp_f_)
    {

      cv::Mat image;
      cv::cvtColor(cv_img_ptr_->image, image, CV_GRAY2BGR);

      int ImgScale = 1; //should be from parameter
      int IDNum;

      double hScale = 0.5;

      int lmTracking = 94; //should be from parameter
      vector<int>* ptpairsCurrent = &ptpairs_msg_.pairs;

      if (ptpairsCurrent->size() > 0)
      {
        for (int i = 0; i < (ptpairsCurrent->size() - 1); i += 2)
        {
          CvSURFPoint * r = (CvSURFPoint*)cvGetSeqElem(keypoints_ptr_, (*ptpairsCurrent)[i + 1]);
          CvPoint center;
          center.x = cvRound(ImgScale * r->pt.x);
          center.y = cvRound(ImgScale * r->pt.y);
          int radius = cvRound(ImgScale * r->size * 1.2 / 9. * 2);
          int font = CV_FONT_HERSHEY_SIMPLEX | CV_FONT_ITALIC;
          int thickness = 1;
          int lineType = 8;
          int shift = 0;

          vector<SURFPoint> *SURFDatabase = db_.storage_;

          IDNum = (*SURFDatabase)[(*ptpairsCurrent)[i]].id;

          cv::putText(image, convertInt(step_).c_str(), cvPoint(10, 20), font, hScale, COLORS[Red]);

          if (IDNum == 0) // new landmark
          {
            cv::putText(image, convertInt(IDNum).c_str(), center, font, hScale, COLORS[LightBlue]);
            cv::circle(image, center, radius, COLORS[LightBlue], thickness, lineType, shift);
            //ROS_INFO("new landmark");
          }
          else if (IDNum == lmTracking) // landmark tracking (cyan)
          {
            cv::putText(image, convertInt(IDNum).c_str(), center, font, hScale * 2, COLORS[Cyan]);
            cv::circle(image, center, radius, COLORS[Cyan], thickness, lineType, shift);
          }
          else if ((*SURFDatabase)[(*ptpairsCurrent)[i]].flag == 0) // re-observed landmark within the last 20 images (yellow)
          {
            cv::putText(image, convertInt(IDNum).c_str(), center, font, hScale, COLORS[Yellow]);
            cv::circle(image, center, radius, COLORS[Yellow], thickness, lineType, shift);
          }
          else if ((*SURFDatabase)[(*ptpairsCurrent)[i]].flag == 1) // re-observed landmark from the last image (green)
          {
            cv::putText(image, convertInt(IDNum).c_str(), center, font, hScale, COLORS[Green]);
            cv::circle(image, center, radius, COLORS[Green], thickness, lineType, shift);
          }
          else if ((*SURFDatabase)[(*ptpairsCurrent)[i]].flag == 2) // re-observed landmark from an image more than 20 iterations old (orange)
          {
            cv::putText(image, convertInt(IDNum).c_str(), center, font, hScale, COLORS[Orange]);
            cv::circle(image, center, radius, COLORS[Orange], thickness, lineType, shift);
          }
          else if ((*SURFDatabase)[(*ptpairsCurrent)[i]].flag == 3) // re-observed landmark from an image more than 40 iterations old (pink)
          {
            cv::putText(image, convertInt(IDNum).c_str(), center, font, hScale, COLORS[Pink]);
            cv::circle(image, center, radius, COLORS[Pink], thickness, lineType, shift);
          }
        }
      }

      if (img_to_file_f_)
      {
        //store db to file
        stringstream _a;
        _a << step_;
        string filename = images_path_ + "log_img_" + _a.str() + ".jpg";
        cv::imwrite(filename.c_str(), image);
      }

      cv::imshow(IMAGE_WINDOW_NAME, image);
      cv::waitKey(3);
    }

  if (ptcloud_pub_f_)
    publishPointCloud();

  if (pose_pub_f_)
    publishPose();

  if (tf_pub_f_)
    publishTransformation();
}

void DataloggerCore::map3DCallback(const sscrovers_pmslam_common::Map3DConstPtr& msg)
{
  map3d_msg_ = *msg;
  if (map3d_to_file_f_)
  {
    stringstream _a;
    _a << msg->header.stamp.nsec;
    string filename = raw_data_path_ + "log_map3d_" + _a.str() + ".log";
    std::ofstream file;
    file.open(filename.c_str());
    //put if here!!
    file << "total[landmarks]: " << msg->landmarks.size() << endl;

    file << "landmarks:\n#\tindex\tx\ty\tz\tmatpos\tstddev" << endl;
    for (unsigned int i = 0; i < msg->landmarks.size(); i++)
    {

      file << i << ": " << msg->landmarks[i].index << "\t" << msg->landmarks[i].position.x << "\t"
          << msg->landmarks[i].position.y << "\t" << msg->landmarks[i].position.z << "\t" << msg->landmarks[i].matpos
          << "\t" << msg->landmarks[i].stddev << endl;
    }
    file.close();
  }
}

void DataloggerCore::pmslamDataCallback(const sscrovers_pmslam_common::PMSlamDataConstPtr& msg)
{
  pmslamdata_msg_ = *msg;
  if (pmslamdata_to_file_f_)
  {
    stringstream _a;
    _a << msg->header.stamp.nsec;
    string filename = raw_data_path_ + "log_pmslamdata_" + _a.str() + ".log";
    std::ofstream file;
    file.open(filename.c_str());
    //put if here!!
    file << "total [map_out]: " << msg->map_out.size() << endl;
    double _roll, _pitch, _yaw;
#if ROS_VERSION_MINIMUM(1, 8, 0) // if current ros version is >= 1.8.0 (fuerte)
    //min. fuerte
    tf::Quaternion _q;
    tf::quaternionMsgToTF(msg->trajectory_out.orientation, _q);
    tf::Matrix3x3(_q).getRPY(_roll, _pitch, _yaw);
#else
    //electric and older
    btQuaternion _q;
    tf::quaternionMsgToTF(msg->trajectory_out.orientation, _q);
    btMatrix3x3(_q).getRPY(_roll, _pitch, _yaw);
#endif
    file << "trajectory_out: #" << step_ << "\tx=" << msg->trajectory_out.position.x << "\ty="
        << msg->trajectory_out.position.y << "\tz=" << msg->trajectory_out.position.z << "\tr=" << _roll << "\tp="
        << _pitch << "\ty=" << _yaw << endl << endl;

    file << "#\tid\tx\ty\tz" << endl;
    for (unsigned int i = 0; i < msg->map_out.size(); i++)
    {

      file << i << ": " << msg->map_out[i].id << "\t" << msg->map_out[i].pt.x << "\t" << msg->map_out[i].pt.y << "\t"
          << msg->map_out[i].pt.z << endl;
    }
    file.close();
  }

geometry_msgs::PoseStamped pt;
  pt.pose.position = msg->trajectory_out.position;
  output_traj_msg.poses.push_back(pt);

  if (pmslamdata_to_file_f_)
  {
    //!!!split this to function for trajectory and point!!!
    //store trajectory to file
    stringstream _a;
    _a << msg->header.stamp.nsec;
    string filename = raw_data_path_ + "log_out_traj_" + _a.str() + ".log";
    std::ofstream file;
    file.open(filename.c_str());
    //put if here!!
    file << "total: " << output_traj_msg.poses.size() << endl;
    file << "#" << step_ << std::endl << std::endl;
    file << "#\tx\ty\tz\tq1\tq2\tq3\tq4\t\troll\tpitch\tyaw" << endl;
    for (unsigned int i = 0; i < output_traj_msg.poses.size(); i++)
    {
      double _roll, _pitch, _yaw;
#if ROS_VERSION_MINIMUM(1, 8, 0) // if current ros version is >= 1.8.0 (fuerte)
      //min. fuerte
      tf::Quaternion _q;
      tf::quaternionMsgToTF(output_traj_msg.poses[i].pose.orientation, _q);
      tf::Matrix3x3(_q).getRPY(_roll, _pitch, _yaw);
#else
      //electric and older
      btQuaternion _q;
      tf::quaternionMsgToTF(output_traj_msg.poses[i].pose.orientation, _q);
      btMatrix3x3(_q).getRPY(_roll, _pitch, _yaw);
#endif

      file << i << ": " << output_traj_msg.poses[i].pose.position.x << "\t" << output_traj_msg.poses[i].pose.position.y
      << "\t" << output_traj_msg.poses[i].pose.position.z << "\t" << output_traj_msg.poses[i].pose.orientation.x << "\t"
      << output_traj_msg.poses[i].pose.orientation.y << "\t" << output_traj_msg.poses[i].pose.orientation.z << "\t"
      << output_traj_msg.poses[i].pose.orientation.w << "\t" << _roll << "\t" << _pitch << "\t" << _yaw << endl;

    }
    file.close();
  }

}

void DataloggerCore::ctrlvecCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  ctrl_vec_ = *msg;
  if (ctrlvec_to_file_f_)
  {
    //store trajectory to file
    stringstream _a;
    _a << msg->header.stamp.nsec;
    string filename = raw_data_path_ + "log_ctrlvec_" + _a.str() + ".log";
    std::ofstream file;
    file.open(filename.c_str());
    //put if here!!
    file << "#\tx\ty\tz\tq1\tq2\tq3\tq4\t\troll\tpitch\tyaw" << endl;

    double _roll, _pitch, _yaw;
#if ROS_VERSION_MINIMUM(1, 8, 0) // if current ros version is >= 1.8.0 (fuerte)
    //min. fuerte
    tf::Quaternion _q;
    tf::quaternionMsgToTF(ctrl_vec_.pose.orientation, _q);
    tf::Matrix3x3(_q).getRPY(_roll, _pitch, _yaw);
#else
    //electric and older
    btQuaternion _q;
    tf::quaternionMsgToTF(ctrl_vec_.pose.orientation, _q);
    btMatrix3x3(_q).getRPY(_roll, _pitch, _yaw);
#endif

    file << "0: " << ctrl_vec_.pose.position.x << "\t" << ctrl_vec_.pose.position.y << "\t" << ctrl_vec_.pose.position.z
        << "\t" << ctrl_vec_.pose.orientation.x << "\t" << ctrl_vec_.pose.orientation.y << "\t"
        << ctrl_vec_.pose.orientation.z << "\t" << ctrl_vec_.pose.orientation.w << "\t" << _roll << "\t" << _pitch
        << "\t" << _yaw << endl;

    file.close();
  }
}

void DataloggerCore::esttrajCallback(const nav_msgs::PathConstPtr& msg)
{
  RoverState curr_pose_;
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
      ROS_WARN("There is no trajectory point corresponding to features data...");
      //data_completed_f_ = false;
    }
  }

  if (esttraj_to_file_f_)
  {
    //!!!split this to function for trajectory and point!!!
    //store trajectory to file
    stringstream _a;
    _a << msg->header.stamp.nsec;
    string filename = raw_data_path_ + "log_esttraj_" + _a.str() + ".log";
    std::ofstream file;
    file.open(filename.c_str());
    //put if here!!
    file << "total: " << est_traj_msg_.poses.size() << endl;
    file << "current pt: #" << step_ << "\tx=" << curr_pose_.x << "\ty=" << curr_pose_.y << "\tz=" << curr_pose_.z
        << "\tr=" << curr_pose_.roll << "\tp=" << curr_pose_.pitch << "\ty=" << curr_pose_.yaw << endl << endl;
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

      file << i << ": " << est_traj_msg_.poses[i].pose.position.x << "\t" << est_traj_msg_.poses[i].pose.position.y
          << "\t" << est_traj_msg_.poses[i].pose.position.z << "\t" << est_traj_msg_.poses[i].pose.orientation.x << "\t"
          << est_traj_msg_.poses[i].pose.orientation.y << "\t" << est_traj_msg_.poses[i].pose.orientation.z << "\t"
          << est_traj_msg_.poses[i].pose.orientation.w << "\t" << _roll << "\t" << _pitch << "\t" << _yaw << endl;
    }
    file.close();
  }
}

void DataloggerCore::intrajCallback(const nav_msgs::PathConstPtr& msg)
{
  RoverState curr_pose_;
  in_traj_msg_ = *msg;
  if (step_ >= 0)
  {
    if ((int)in_traj_msg_.poses.size() > step_)
    {
      curr_pose_.x = in_traj_msg_.poses[step_].pose.position.x;
      curr_pose_.y = in_traj_msg_.poses[step_].pose.position.y;
      curr_pose_.z = in_traj_msg_.poses[step_].pose.position.z;

      double _roll, _pitch, _yaw;
#if ROS_VERSION_MINIMUM(1, 8, 0) // if current ros version is >= 1.8.0 (fuerte)
      //min. fuerte
      tf::Quaternion _q;
      tf::quaternionMsgToTF(in_traj_msg_.poses[step_].pose.orientation, _q);
      tf::Matrix3x3(_q).getRPY(_roll, _pitch, _yaw);
#else
      //electric and older
      btQuaternion _q;
      tf::quaternionMsgToTF(in_traj_msg_.poses[step_].pose.orientation, _q);
      btMatrix3x3(_q).getRPY(_roll, _pitch, _yaw);
#endif

      curr_pose_.roll = _roll;
      curr_pose_.pitch = _pitch;
      curr_pose_.yaw = _yaw;

    }
    else
    {
      ROS_WARN("There is no trajectory point corresponding to features data...");
    }
  }

  if (intraj_to_file_f_)
  {
    //!!!split this to function for trajectory and point!!!
    //store trajectory to file
    stringstream _a;
    _a << msg->header.stamp.nsec;
    string filename = raw_data_path_ + "log_intraj_" + _a.str() + ".log";
    std::ofstream file;
    file.open(filename.c_str());
    //put if here!!
    file << "total: " << in_traj_msg_.poses.size() << endl;
    file << "current pt: #" << step_ << "\tx=" << curr_pose_.x << "\ty=" << curr_pose_.y << "\tz=" << curr_pose_.z
        << "\tr=" << curr_pose_.roll << "\tp=" << curr_pose_.pitch << "\ty=" << curr_pose_.yaw << endl << endl;
    file << "#\tx\ty\tz\tq1\tq2\tq3\tq4\t\troll\tpitch\tyaw" << endl;
    for (unsigned int i = 0; i < in_traj_msg_.poses.size(); i++)
    {
      double _roll, _pitch, _yaw;
#if ROS_VERSION_MINIMUM(1, 8, 0) // if current ros version is >= 1.8.0 (fuerte)
      //min. fuerte
      tf::Quaternion _q;
      tf::quaternionMsgToTF(in_traj_msg_.poses[i].pose.orientation, _q);
      tf::Matrix3x3(_q).getRPY(_roll, _pitch, _yaw);
#else
      //electric and older
      btQuaternion _q;
      tf::quaternionMsgToTF(in_traj_msg_.poses[i].pose.orientation, _q);
      btMatrix3x3(_q).getRPY(_roll, _pitch, _yaw);
#endif

      file << i << ": " << in_traj_msg_.poses[i].pose.position.x << "\t" << in_traj_msg_.poses[i].pose.position.y
          << "\t" << in_traj_msg_.poses[i].pose.position.z << "\t" << in_traj_msg_.poses[i].pose.orientation.x << "\t"
          << in_traj_msg_.poses[i].pose.orientation.y << "\t" << in_traj_msg_.poses[i].pose.orientation.z << "\t"
          << in_traj_msg_.poses[i].pose.orientation.w << "\t" << _roll << "\t" << _pitch << "\t" << _yaw << endl;
    }
    file.close();
  }
}

void DataloggerCore::truetrajCallback(const nav_msgs::PathConstPtr& msg)
{
  RoverState curr_pose_;
  true_traj_msg_ = *msg;
  if (step_ >= 0)
  {
    if ((int)true_traj_msg_.poses.size() > step_)
    {
      curr_pose_.x = true_traj_msg_.poses[step_].pose.position.x;
      curr_pose_.y = true_traj_msg_.poses[step_].pose.position.y;
      curr_pose_.z = true_traj_msg_.poses[step_].pose.position.z;

      double _roll, _pitch, _yaw;
#if ROS_VERSION_MINIMUM(1, 8, 0) // if current ros version is >= 1.8.0 (fuerte)
      //min. fuerte
      tf::Quaternion _q;
      tf::quaternionMsgToTF(true_traj_msg_.poses[step_].pose.orientation, _q);
      tf::Matrix3x3(_q).getRPY(_roll, _pitch, _yaw);
#else
      //electric and older
      btQuaternion _q;
      tf::quaternionMsgToTF(true_traj_msg_.poses[step_].pose.orientation, _q);
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

  if (truetraj_to_file_f_)
  {
    //!!!split this to function for trajectory and point!!!
    //store trajectory to file
    stringstream _a;
    _a << msg->header.stamp.nsec;
    string filename = raw_data_path_ + "log_truetraj_" + _a.str() + ".log";
    std::ofstream file;
    file.open(filename.c_str());
    //put if here!!
    file << "total: " << true_traj_msg_.poses.size() << endl;
    file << "current pt: #" << step_ << "\tx=" << curr_pose_.x << "\ty=" << curr_pose_.y << "\tz=" << curr_pose_.z
        << "\tr=" << curr_pose_.roll << "\tp=" << curr_pose_.pitch << "\ty=" << curr_pose_.yaw << endl << endl;
    file << "#\tx\ty\tz\tq1\tq2\tq3\tq4\t\troll\tpitch\tyaw" << endl;
    for (unsigned int i = 0; i < true_traj_msg_.poses.size(); i++)
    {
      double _roll, _pitch, _yaw;
#if ROS_VERSION_MINIMUM(1, 8, 0) // if current ros version is >= 1.8.0 (fuerte)
      //min. fuerte
      tf::Quaternion _q;
      tf::quaternionMsgToTF(true_traj_msg_.poses[i].pose.orientation, _q);
      tf::Matrix3x3(_q).getRPY(_roll, _pitch, _yaw);
#else
      //electric and older
      btQuaternion _q;
      tf::quaternionMsgToTF(true_traj_msg_.poses[i].pose.orientation, _q);
      btMatrix3x3(_q).getRPY(_roll, _pitch, _yaw);
#endif

      file << i << ": " << true_traj_msg_.poses[i].pose.position.x << "\t" << true_traj_msg_.poses[i].pose.position.y
          << "\t" << true_traj_msg_.poses[i].pose.position.z << "\t" << true_traj_msg_.poses[i].pose.orientation.x
          << "\t" << true_traj_msg_.poses[i].pose.orientation.y << "\t" << true_traj_msg_.poses[i].pose.orientation.z
          << "\t" << true_traj_msg_.poses[i].pose.orientation.w << "\t" << _roll << "\t" << _pitch << "\t" << _yaw
          << endl;
    }
    file.close();
  }
}

void DataloggerCore::ptpairs3dCallback(const sscrovers_pmslam_common::PairedPoints3DConstPtr& msg)
{
  //step_ = msg->header.stamp.nsec;
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

  if (points3d_to_file_f_)
  {
    //store descriptors to file
    stringstream _a;
    _a << msg->header.stamp.nsec;
    string filename = raw_data_path_ + "log_pt3d_" + _a.str() + "";
    std::ofstream file;
    file.open(filename.c_str());
    //put if here!!
    file << "total: " << msg->pts.size() << endl;
    for (unsigned int i = 0; i < msg->pts.size(); i++)
    {
      file << i << ": " << ptpairs_[i] << "\t" << points3d_[i].x << "\t" << points3d_[i].y << "\t" << points3d_[i].z
          << endl;
    }
    file.close();
  }
}

void DataloggerCore::featuresDBCallback(const sscrovers_pmslam_common::DynamicArrayConstPtr& msg)
{
  db_nsec = msg->header.stamp.nsec;
  if (msg->dims[0] > 0)
  {
    db_.storage_->resize(msg->dims[0]);
    memcpy(db_.storage_->data(), msg->data.data(), msg->dims[0] * sizeof(SURFPoint));
    db_update_f_ = true;
  }
  else
    ROS_ERROR("No data in database topic");

  if (db_to_file_f_)
  {
    //store db to file
    stringstream _a;
    _a << msg->header.stamp.nsec;
    string filename = raw_data_path_ + "log_db_" + _a.str() + ".log";
    std::ofstream file;
    file.open(filename.c_str());
    //put if here!!
    file << "total: " << db_.storage_->size() << endl;
    file << "#\t index\n" << "pt.pt.x\t pt.pt.y\t lap\t size\t dir\t hess\n"
        << "state.x\t state.y\t state.z\t state.yaw\t state.pitch\t state.roll\n" << "pxLocation[0]\t pxLocation[1]\n"
        << "gtruth[0]\t gtruth[1]\t gtruth[2]\n" << "n\t id\t step\t flag\n" << "desc[64]" << endl << endl;
    for (unsigned int i = 0; i < db_.storage_->size(); i++)
    {
      vector<SURFPoint> _vec;
      _vec = *db_.storage_;

      file << i << ": " << _vec[i].index << endl << _vec[i].pt.pt.x << "\t" << _vec[i].pt.pt.y << "\t"
          << _vec[i].pt.laplacian << "\t" << _vec[i].pt.size << "\t" << _vec[i].pt.dir << "\t" << _vec[i].pt.hessian
          << endl << _vec[i].state.x << "\t" << _vec[i].state.y << "\t" << _vec[i].state.z << "\t" << _vec[i].state.yaw
          << "\t" << _vec[i].state.pitch << "\t" << _vec[i].state.roll << endl << _vec[i].pxLocation[0] << "\t"
          << _vec[i].pxLocation[1] << endl << _vec[i].gtruth[0] << "\t" << _vec[i].gtruth[1] << "\t"
          << _vec[i].gtruth[2] << endl << "\t" << _vec[i].n << "\t" << _vec[i].id << "\t" << _vec[i].step << "\t"
          << _vec[i].flag << endl;

      for (int j = 0; j < 64; j++)
      {
        file << _vec[i].descriptor[j] << "\t";
      }
      file << endl << endl;
    }
    file.close();
  }
}

void DataloggerCore::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  img_nsec = msg->header.stamp.nsec;
  try
  {
    cv_img_ptr_ = cv_bridge::toCvCopy(msg, enc::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
}

void DataloggerCore::ptpairsCallback(const sscrovers_pmslam_common::PtPairsConstPtr& msg)
{
  //step_ = msg->header.stamp.nsec;
  ptpairs_msg_ = *msg;
  pt_nsec = msg->header.stamp.nsec;

  if (ptpairs_to_file_f_)
  {
    //store keypoints to file
    stringstream _a;
    _a << msg->header.stamp.nsec;
    string filename = raw_data_path_ + "log_ptpoints_" + _a.str() + ".log";
    std::ofstream file;
    file.open(filename.c_str());
    //put if here!!
    file << "total: " << ptpairs_msg_.pairs.size() << endl;
    file << "#\tid" << endl;
    for (unsigned int i = 0; i < ptpairs_msg_.pairs.size(); i++)
    {
      file << i << ": \t" << ptpairs_msg_.pairs[i] << endl;
    }
    file.close();
  }
}

void DataloggerCore::featuresCallback(const sscrovers_pmslam_common::DynamicArrayConstPtr& msg)
{
  step_ = msg->header.stamp.nsec;
  kp_nsec = msg->header.stamp.nsec;

  unsigned int kp_size = msg->dims[0] * msg->dims[1] * sizeof(CvSURFPoint);
  //unsigned int ds_size = msg->dims[0] * msg->dims[2] * sizeof(float);

  CvMemStorage *mem_storage_kp = cvCreateMemStorage(0);
  keypoints_ptr_ = cvCreateSeq(0, sizeof(CvSeq), sizeof(CvSURFPoint), mem_storage_kp);
  cvSeqPushMulti(keypoints_ptr_, (CvSURFPoint*)&msg->data[0], msg->dims[0]);

  CvMemStorage *mem_storage_dscp = cvCreateMemStorage(0);
  descriptors_ptr_ = cvCreateSeq(0, sizeof(CvSeq), sizeof(float) * msg->dims[2], mem_storage_dscp);

  typedef struct float64
  {
    float x[64];
  } float64;
  cvSeqPushMulti(descriptors_ptr_, (float64*)(&msg->data[0] + kp_size), msg->dims[0]);

  if (features_to_file_f_)
  {
    //store keypoints to file
    stringstream _a1;
    _a1 << step_;
    string filename1 = raw_data_path_ + "log_keypoints_" + _a1.str() + ".log";
    std::ofstream file1;
    file1.open(filename1.c_str());
    //put if here!!
    file1 << "total: " << keypoints_ptr_->total << endl;
    file1 << "#\tpt.x\tpt.y\tlap\tsize\tdir\thess" << endl;
    for (int i = 0; i < keypoints_ptr_->total; i++)
    {
      CvSURFPoint * r = (CvSURFPoint*)cvGetSeqElem(keypoints_ptr_, i);
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
    string filename = raw_data_path_ + "log_descriptors_" + _a.str() + ".log";
    std::ofstream file;
    file.open(filename.c_str());
    //put if here!!
    file << "total: " << descriptors_ptr_->total << endl;
    file << "#\tpt.x\tpt.y\tlap\tsize\tdir\thess" << endl;
    int _s = descriptors_ptr_->elem_size / sizeof(float);
    for (int i = 0; i < descriptors_ptr_->total; i++)
    {
      float* r = (float*)cvGetSeqElem(descriptors_ptr_, i);
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

/*
 void DataloggerCore::configCallback(node_example::node_example_paramsConfig &config, uint32_t level)
 {
 // Set class variables to new values. They should match what is input at the dynamic reconfigure GUI.
 message = config.message.c_str();
 a = config.a;
 b = config.b;
 } // end configCallback()
 */
void DataloggerCore::publishPose()
{
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header.stamp.nsec = 0; //set up header
  pose_msg.pose = pmslamdata_msg_.trajectory_out;
  pose_pub_.publish(pose_msg);
}

void DataloggerCore::publishTransformation()
{
  /*//publish transformation for current position of observer
   tf::TransformBroadcaster tfB_;
   ROS_ASSERT(tfB_);

   tf::Transform map_to_odom_;

   //init - constructor//:map_to_odom_(tf::Transform(tf::createQuaternionFromRPY( 0, 0, 0 ), tf::Point(0, 0, 0 )))l;

   tf::Stamped<tf::Pose> odom_to_map;
   try
   {
   tf_.transformPose(odom_frame_,tf::Stamped<tf::Pose> (tf::Transform(tf::createQuaternionFromRPY(0, 0, mpose.theta),
   tf::Vector3(mpose.x, mpose.y, 0.0)).inverse(),
   scan->header.stamp, base_frame_),odom_to_map);
   }
   catch(tf::TransformException e){
   ROS_ERROR("Transform from base_link to odom failed\n");
   odom_to_map.setIdentity();
   }

   map_to_odom_mutex_.lock();
   map_to_odom_ = tf::Transform(tf::Quaternion( odom_to_map.getRotation() ),
   tf::Point(      odom_to_map.getOrigin() ) ).inverse();
   map_to_odom_mutex_.unlock();

   //tfB_.sendTransform( tf::StampedTransform (map_to_odom_, ros::Time::now(), map_frame_, odom_frame_));
   */
}

void DataloggerCore::publishPointCloud()
{
  //pmslam_data as a source
  ptcloud_.header.frame_id = "/";
  ptcloud_.width = ptcloud_.height = 1;
  ptcloud_.header.stamp = ros::Time::now();
  ptcloud_.resize(pmslamdata_msg_.map_out.size());
  for (unsigned int i = 0; i < pmslamdata_msg_.map_out.size(); i++)
  {
    ptcloud_[i].x = pmslamdata_msg_.map_out[i].pt.x;
    ptcloud_[i].y = pmslamdata_msg_.map_out[i].pt.y;
    ptcloud_[i].z = pmslamdata_msg_.map_out[i].pt.z;

    uchar pixel = 0;
    unsigned int _id = pmslamdata_msg_.map_out[i].id;
    if (_id < (*db_.storage_).size())
      pixel = cv_img_ptr_->image.at < uchar > ((*db_.storage_)[_id].pt.pt.x, (*db_.storage_)[_id].pt.pt.y);
    else
      pixel = 255;

    uint8_t r = 0, g = 0, b = pixel; // Example: black color
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    ptcloud_[i].rgb = *reinterpret_cast<float*>(&rgb);
  }

  /*//map3d as a source
   ptcloud_.resize(map3d_msg_.landmarks.size());
   for (unsigned int i = 0; i < map3d_msg_.landmarks.size(); i++)
   {
   ptcloud_[i].x = map3d_msg_.landmarks[i].position.x;
   ptcloud_[i].y = map3d_msg_.landmarks[i].position.y;
   ptcloud_[i].z = map3d_msg_.landmarks[i].position.z;

   uint8_t r = 0, g = 0, b = 0; // Example: black color
   uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
   ptcloud_[i].rgb = *reinterpret_cast<float*>(&rgb);
   }
   */
  //ptcloud_pub_.publish(ptcloud_.makeShared());
  ptcloud_pub_.publish(ptcloud_);
}

//------------------------------------------MAIN-------------------------------------------
int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "pmslam_datalogger");
  ros::NodeHandle n;

  // Create a new NodeExample object.
  DataloggerCore *dt_core = new DataloggerCore(&n);

  // Tell ROS how fast to run this node.
  ros::Rate r(dt_core->rate_);

  // Main loop.

  while (n.ok())
  {
    dt_core->process();
    //dt_core->Publish();
    ros::spinOnce();
    r.sleep();
  }

  //ros::spin();

  return 0;
} // end main()
